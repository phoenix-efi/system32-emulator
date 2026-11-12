/* bench_engine.h -- the engine side of the bench: what a real engine and its instruments would make of the ECUs'
 * output pins. Nothing here knows anything about the firmware; it sees only injector pin edges, the engine model's
 * RPM and the sensor values the bench itself is applying.
 *
 *  inj_meter   a per-channel injector logic analyser. Each injector output channel (RA0..RA15) of each ECU feeds it
 *              edges; the two ECUs' drivers are wired in parallel to the same injector, so the injector is open while
 *              EITHER driver is on (union). Each union opening delivers fuel for (open time - dead time) x flow rate.
 *  bench_air   speed-density air flow of a four-stroke piston engine: VE x displacement x RPM/2 x MAP/(R T_IAT).
 *  plx_smafr   the PLX SM-AFR wideband module's linear analogue output (gasoline): 0 V = 10.0 AFR, 5 V = 20.0 AFR
 *              (PLX: 0-5 V linear output, 10-20 AFR gasoline range).
 *
 * Six-cylinder defaults (bench_engine_six): a 540 cu in six-cylinder engine, 60 lb/hr injectors, 0.80 ms injector dead
 * time at the bench's 13.8 V supply (a typical high-impedance injector), rising as the supply falls (inj_dead_factor:
 * the usual shape of a high-impedance injector's published offset-vs-battery curve, INFERRED; no data sheet for the
 * installed injector), VE 0.868 -- calibrated once so a six-cylinder configuration's maps read AFR 13.3 at 2400 RPM /
 * 24.0 inHg / IAT 59 F / trim 0 (docs/simulator.md, "Wideband AFR").
 */
#ifndef BENCH_ENGINE_H
#define BENCH_ENGINE_H
#include <stdbool.h>
#include <stdint.h>

#define INJ_SRC 2         /* two ECUs */
#define INJ_CH 16         /* PORTA bits */
#define INJ_QUEUE 32      /* completed pulses per source+channel awaiting the union merge */

typedef struct { uint64_t open, close; double volts; } inj_pulse;   /* volts: injector supply when it opened */

typedef struct {
  double hz;                                   /* cycle clock of the edge timestamps */
  double dead_s;                               /* injector dead time per opening at INJ_DEAD_REF_V */
  bool dead_fixed;                             /* true: dead_s at any voltage (no supply dependence) */
  double volts;                                /* injector supply (the aircraft bus) now, set by the harness */
  double flow_gps;                             /* static flow per injector, g/s */
  /* per source: written by that source's edge callback only (safe across the ECU threads) */
  bool enabled[INJ_SRC];                       /* injector-enable switch per source (default on) */
  bool is_open[INJ_SRC][INJ_CH];
  uint64_t open_at[INJ_SRC][INJ_CH];
  double open_v[INJ_SRC][INJ_CH];
  inj_pulse q[INJ_SRC][INJ_CH][INJ_QUEUE];
  unsigned qn[INJ_SRC][INJ_CH];
  /* merged (single-threaded: inj_meter_settle) */
  uint64_t union_pulses[INJ_CH];
  double union_open_s[INJ_CH];                 /* total open time of the union pulses per channel */
  double grams_total;                          /* fuel delivered since reset, all channels */
  double last_net_s[INJ_CH];                   /* net (open - dead) of the newest union pulse */
  uint64_t last_open[INJ_CH], prev_open[INJ_CH];
} inj_meter;

#define INJ_DEAD_REF_V 13.8   /* supply at which the dead time equals inj_dead_ms */
#define INJ_OPEN_MIN_V 6.0    /* below this supply the injector does not open at all (INFERRED) */
void inj_meter_init(inj_meter *m, double hz, double dead_ms, double flow_lb_hr);
/* Dead time at `volts` relative to the dead time at INJ_DEAD_REF_V (1.0 there); +inf below INJ_OPEN_MIN_V. Piecewise
 * linear through 6 V 4.00, 8 V 2.38, 10 V 1.63, 12 V 1.23, 13.8 V 1.00, 16 V 0.80, 18 V 0.69 (flat above): a typical
 * high-impedance injector's curve normalised to its 13.8 V value (INFERRED). */
double inj_dead_factor(double volts);
/* One output-latch change of source `src` at cycle `cyc` (old -> new PORTA latch). Call from that source's thread. */
void inj_meter_edge(inj_meter *m, int src, uint64_t cyc, uint16_t old_lat, uint16_t new_lat);
/* Source lost power: a driver that is off closes the injector; pulses still open from it are dropped. */
void inj_meter_drop_source(inj_meter *m, int src);
/* The cockpit's per-ECU injector-enable switch: with it off, that ECU's drivers do not reach the injectors (its output
 * latch may still toggle -- OBSERVED on the stock firmware, which keeps pulsing PORTA with RC13 low -- so the switch
 * is modelled as gating the drive, INFERRED). Turning it off drops pulses still open from that source. */
void inj_meter_enable_source(inj_meter *m, int src, bool on);
/* Merge both sources' completed pulses into union pulses, up to `now` (every source has been run to `now`).
 * Single-threaded (between quanta). */
void inj_meter_settle(inj_meter *m, uint64_t now);
/* Fuel flow (g/s) estimated from each channel's newest union pulse and its repetition period; a channel silent for
 * more than `stale_s` (or two periods) counts as 0. */
double inj_meter_fuel_gps(const inj_meter *m, uint64_t now, double stale_s);

typedef struct {
  double displacement_ci;  /* swept volume, cubic inches */
  double ve;               /* volumetric efficiency, 0..1 */
  double inj_flow_lb_hr;   /* per injector */
  double inj_dead_ms;
} bench_engine;

bench_engine bench_engine_six(void);
/* Air mass flow, g/s, of a four-stroke engine: VE * Vd * (rpm / 120) * rho(MAP, IAT). */
double bench_air_gps(const bench_engine *e, double rpm, double map_inhg, double iat_f);
/* AFR from air and fuel flow; `lean_limit` when there is no fuel. */
double bench_afr(double air_gps, double fuel_gps, double lean_limit);
/* PLX SM-AFR linear output, gasoline: volts for an AFR, saturating at 0..5 V. */
double plx_smafr_volts(double afr);
double plx_smafr_afr(double volts);
/* First-order sensor lag: the value a sensor with time constant tau shows dt after reading `prev`. */
double bench_lag(double prev, double target, double dt_s, double tau_s);

#endif
