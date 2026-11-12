/* xp_rref.c -- see xp_rref.h. */
#include "xp_rref.h"
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

int xp_open(xp_rref *x, const char *host, int port) {
  memset(x, 0, sizeof *x);
  strncpy(x->host, host, sizeof x->host - 1); x->port = port;
  x->sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (x->sock < 0) return -1;
  struct sockaddr_in me = {0};
  me.sin_family = AF_INET; me.sin_addr.s_addr = htonl(INADDR_ANY); me.sin_port = 0;   /* any local port: X-Plane replies to it */
  if (bind(x->sock, (struct sockaddr *)&me, sizeof me)) return -2;
  fcntl(x->sock, F_SETFL, fcntl(x->sock, F_GETFL) | O_NONBLOCK);
  return 0;
}

int xp_add(xp_rref *x, const char *dataref) { if (x->n >= XP_MAX) return -1; x->names[x->n] = dataref; return x->n++; }

void xp_subscribe(xp_rref *x, int hz) {
  struct sockaddr_in to = {0};
  to.sin_family = AF_INET; to.sin_port = htons((uint16_t)x->port); inet_pton(AF_INET, x->host, &to.sin_addr);
  for (int i = 0; i < x->n; i++) {
    unsigned char pkt[5 + 4 + 4 + 400] = {0};
    memcpy(pkt, "RREF", 5);
    int32_t f = hz, idx = i;
    memcpy(pkt + 5, &f, 4); memcpy(pkt + 9, &idx, 4);
    strncpy((char *)pkt + 13, x->names[i], 399);
    sendto(x->sock, pkt, sizeof pkt, 0, (struct sockaddr *)&to, sizeof to);
  }
}

int xp_poll(xp_rref *x, double now_wall) {
  unsigned char buf[2048]; int updated = 0;
  for (;;) {
    ssize_t n = recv(x->sock, buf, sizeof buf, 0);
    if (n < 5) break;
    if (memcmp(buf, "RREF", 4)) continue;
    x->packets++; x->last_rx = now_wall;
    for (ssize_t o = 5; o + 8 <= n; o += 8) {
      int32_t idx; float v; memcpy(&idx, buf + o, 4); memcpy(&v, buf + o + 4, 4);
      if (idx >= 0 && idx < x->n) { x->val[idx] = v; x->have[idx] = true; updated++; }
    }
  }
  return updated;
}

void xp_close(xp_rref *x) { if (x->sock > 0) close(x->sock); x->sock = -1; }
