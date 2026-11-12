/* xp_rref.h -- minimal X-Plane UDP dataref subscriber (RREF), C, non-blocking; one poll per co-simulation quantum.
 * Request: "RREF\0" + int32 freq_hz + int32 index + char[400] dataref; reply: "RREF," + n x {int32 index, float value}. */
#ifndef XP_RREF_H
#define XP_RREF_H
#include <stdbool.h>

#define XP_MAX 16
typedef struct {
  int sock;
  char host[64]; int port;
  const char *names[XP_MAX]; int n;
  float val[XP_MAX]; bool have[XP_MAX];
  double last_rx;            /* wall seconds of the last packet (0 = never) */
  unsigned long packets;
} xp_rref;

int xp_open(xp_rref *x, const char *host, int port);        /* 0 = ok */
int xp_add(xp_rref *x, const char *dataref);                 /* index */
void xp_subscribe(xp_rref *x, int hz);                       /* (re)send every subscription */
int xp_poll(xp_rref *x, double now_wall);                    /* drain pending packets; returns values updated */
void xp_close(xp_rref *x);

#endif
