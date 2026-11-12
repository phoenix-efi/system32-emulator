/* bench_engine.c -- see bench_engine.h. */
#include "bench_engine.h"
#include <math.h>
#include <string.h>

#define LB_HR_TO_GPS (453.59237 / 3600.0)
#define CI_TO_M3 1.6387064e-5
#define INHG_TO_PA 3386.389
#define R_AIR 287.05            /* J/(kg K), dry air */

void inj_meter_init(inj_meter *m, double hz, double dead_ms, double flow_lb_hr) {
  memset(m, 0, sizeof *m);
  m->hz = hz;
  m->dead_s = dead_ms / 1e3;
  m->flow_gps = flow_lb_hr * LB_HR_TO_GPS;
  m->volts = INJ_DEAD_REF_V;
  for (int s = 0; s < INJ_SRC; s++) m->enabled[s] = true;
}

void inj_meter_edge(inj_meter *m, int src, uint64_t cyc, uint16_t old_lat, uint16_t new_lat) {
  if (src < 0 || src >= INJ_SRC || !m->enabled[src]) return;
  uint16_t ch = (uint16_t)(old_lat ^ new_lat);
  for (unsigned b = 0; b < INJ_CH; b++) {
    if (!((ch >> b) & 1u)) continue;
    if ((new_lat >> b) & 1u) {
      m->is_open[src][b] = true;
      m->open_at[src][b] = cyc;
      m->open_v[src][b] = m->volts;
    } else if (m->is_open[src][b]) {
      m->is_open[src][b] = false;
      unsigned *n = &m->qn[src][b];
      if (*n < INJ_QUEUE) { m->q[src][b][*n] = (inj_pulse){.open = m->open_at[src][b], .close = cyc, .volts = m->open_v[src][b]}; (*n)++; }
    }
  }
}

void inj_meter_drop_source(inj_meter *m, int src) {
  if (src < 0 || src >= INJ_SRC) return;
  for (unsigned b = 0; b < INJ_CH; b++) m->is_open[src][b] = false;
}

void inj_meter_enable_source(inj_meter *m, int src, bool on) {
  if (src < 0 || src >= INJ_SRC) return;
  if (!on) inj_meter_drop_source(m, src);
  m->enabled[src] = on;
}

double inj_dead_factor(double volts) {
  static const double V[] = {6.0, 8.0, 10.0, 12.0, 13.8, 16.0, 18.0}, F[] = {4.00, 2.38, 1.63, 1.23, 1.00, 0.80, 0.69};
  if (!(volts >= INJ_OPEN_MIN_V)) return INFINITY;
  for (unsigned i = 1; i < sizeof V / sizeof V[0]; i++)
    if (volts <= V[i]) return F[i - 1] + (F[i] - F[i - 1]) * (volts - V[i - 1]) / (V[i] - V[i - 1]);
  return F[sizeof F / sizeof F[0] - 1];
}

static void finalize(inj_meter *m, unsigned b, uint64_t open, uint64_t close, double volts) {
  double dead = m->dead_fixed ? m->dead_s : m->dead_s * inj_dead_factor(volts);
  double net = (double)(close - open) / m->hz - dead;
  if (net < 0.0) net = 0.0;
  m->grams_total += net * m->flow_gps;
  m->prev_open[b] = m->last_open[b];
  m->last_open[b] = open;
  m->last_net_s[b] = net;
  m->union_pulses[b]++;
  m->union_open_s[b] += (double)(close - open) / m->hz;
}

void inj_meter_settle(inj_meter *m, uint64_t now) {
  for (unsigned b = 0; b < INJ_CH; b++) {
    if (!m->qn[0][b] && !m->qn[1][b]) continue;
    uint64_t horizon = now;   /* nothing that is still to come can start before this */
    for (int s = 0; s < INJ_SRC; s++)
      if (m->is_open[s][b] && m->open_at[s][b] < horizon) horizon = m->open_at[s][b];
    /* both queues are in time order: merge them by opening time into union intervals */
    unsigned i[INJ_SRC] = {0, 0};
    for (;;) {
      int pick = -1;
      for (int s = 0; s < INJ_SRC; s++)
        if (i[s] < m->qn[s][b] && (pick < 0 || m->q[s][b][i[s]].open < m->q[pick][b][i[pick]].open)) pick = s;
      if (pick < 0) break;
      unsigned j[INJ_SRC] = {i[0], i[1]};
      uint64_t uo = m->q[pick][b][j[pick]].open, uc = m->q[pick][b][j[pick]].close;
      double uv = m->q[pick][b][j[pick]].volts;
      j[pick]++;
      for (bool grew = true; grew;) {        /* absorb every pulse (either source) that overlaps [uo, uc] */
        grew = false;
        for (int s = 0; s < INJ_SRC; s++)
          while (j[s] < m->qn[s][b] && m->q[s][b][j[s]].open <= uc) {
            if (m->q[s][b][j[s]].close > uc) uc = m->q[s][b][j[s]].close;
            j[s]++;
            grew = true;
          }
      }
      if (uc >= horizon) break;               /* an open or future pulse may still join it */
      finalize(m, b, uo, uc, uv);
      i[0] = j[0];
      i[1] = j[1];
    }
    for (int s = 0; s < INJ_SRC; s++) {
      memmove(m->q[s][b], m->q[s][b] + i[s], (m->qn[s][b] - i[s]) * sizeof(inj_pulse));
      m->qn[s][b] -= i[s];
    }
  }
}

double inj_meter_fuel_gps(const inj_meter *m, uint64_t now, double stale_s) {
  double g = 0.0;
  for (unsigned b = 0; b < INJ_CH; b++) {
    if (m->union_pulses[b] < 2 || m->last_open[b] <= m->prev_open[b]) continue;
    double period = (double)(m->last_open[b] - m->prev_open[b]) / m->hz;
    double age = now > m->last_open[b] ? (double)(now - m->last_open[b]) / m->hz : 0.0;
    if (age > stale_s || age > 2.0 * period) continue;
    g += m->last_net_s[b] * m->flow_gps / period;
  }
  return g;
}

bench_engine bench_engine_six(void) {
  bench_engine e = {.displacement_ci = 540.0, .ve = 0.868, .inj_flow_lb_hr = 60.0, .inj_dead_ms = 0.80};
  return e;
}

double bench_air_gps(const bench_engine *e, double rpm, double map_inhg, double iat_f) {
  if (rpm <= 0.0 || map_inhg <= 0.0) return 0.0;
  double t_k = (iat_f - 32.0) * 5.0 / 9.0 + 273.15;
  double rho = map_inhg * INHG_TO_PA / (R_AIR * t_k);            /* kg/m^3 */
  double vd = e->displacement_ci * CI_TO_M3;                      /* m^3 per two revolutions */
  return e->ve * vd * (rpm / 120.0) * rho * 1e3;                  /* g/s */
}

double bench_afr(double air_gps, double fuel_gps, double lean_limit) {
  if (fuel_gps <= 1e-9) return lean_limit;
  double a = air_gps / fuel_gps;
  return a > lean_limit ? lean_limit : a;
}

double plx_smafr_volts(double afr) {
  double v = (afr - 10.0) / 2.0;
  return v < 0.0 ? 0.0 : v > 5.0 ? 5.0 : v;
}

double plx_smafr_afr(double volts) { return 10.0 + 2.0 * volts; }

double bench_lag(double prev, double target, double dt_s, double tau_s) {
  if (tau_s <= 0.0) return target;
  return prev + (target - prev) * (1.0 - exp(-dt_s / tau_s));
}
