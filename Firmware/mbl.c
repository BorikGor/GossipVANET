/*
 * WSAN Mobile (Contiki-NG, Sky/MSPSim)
 *
 * Added in this version:
 * - Periodic 'REQ_LOC' over UART1 (Sky) and parsing 'LOC id x y ts'.
 * - Keep position (x_dm, y_dm), last timestamp, compute speed (dm/s)
 *   and discrete 8-way direction from delta.
 * - Sudden-stop detector: if dx+dy <= 1 dm during >= 2000 ms,
 *   declare Emergency Braking -> turn RED LED ON and send EF.
 *   When motion resumes -> send EF_FINISH (255) and turn LED OFF.
 * - Include motion metrics (x,y,v,dir) in payload of own DATA
 *   (both gossip DATA and unicast-to-RSU) and in EF/EF_FINISH.
 *
 * Existing behavior:
 * - QUERY every WSAN_T_QUERY_MS, RSU 3-misses rule.
 * - Dedup + carry ring, flush to RSU on visibility.
 * - EF re-broadcast on receive and LED reflect remote EF code.
 *
 * NEW in this revision (EF-in-QUERY, multi-EF awareness):
 * - Maintain a small local registry of active emergencies (by origin).
 * - Put EF_flag (1B) + EF_origin_id (4B, BE) into each outgoing QUERY:
 *   "last known" active EF; if none -> (0, 0).
 * - Parse incoming QUERY from mobiles and update EF registry:
 *   EF=0 -> no change; EF=255 -> remove by EF_origin_id; otherwise add.
 * - Refresh LED from EF registry so multiple EF are handled correctly.
 * - Fix: after local FINISH LED turns OFF (ef_led_set(0)).
 *
 * NEW in this patch:
 * - ef_self_active(): guard against duplicate/self EF.
 * - Suppress issuing local EF when we are stopped due to foreign EF.
 * - Debug prints for EF start/finish and LOC parsing.
 */

#include "contiki.h"
#include "net/ipv6/simple-udp.h"
#include "os/sys/etimer.h"
#include "os/sys/ctimer.h"
#include "dev/serial-line.h"
#include "dev/uart1.h"
#include "dev/leds.h"
#include "random.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "wsan_common.h"     /* WSAN_EF_* constants */
#include "wsan_protocol.h"   /* pack/parse, dedup, carry API */

/* ------------ Build-time diagnostics (printf) ------------------- */
#ifndef WSAN_DIAG
#define WSAN_DIAG 0 /* 1=print DATA_RX/CARRY_* diagnostics */
#endif

/* ------------ UDP connection ----------------------------------- */
static struct simple_udp_connection udp;

/* ------------ Node & RSU state --------------------------------- */
static uint8_t  node_id8;
static uint32_t origin_id;
static uint16_t seq16 = 0;
static uip_ipaddr_t rsu_ip;
static uint32_t rsu_id = 0;
static uint8_t  have_rsu = 0;

/* ------------ QUERY/ACK epochs (3-misses rule) ----------------- */
static uint32_t probe_epoch    = 0; /* increment on each QUERY */
static uint32_t ack_epoch_last = 0; /* epoch of last RSU/ACK */

/* ------------ Dedup & carry store (ring) ----------------------- */
static wsan_dedup_t dedup;
static wsan_carry_t carry;
static uint32_t     carry_evicted = 0;

/* ------------ LOC (position/time) & motion metrics ------------- */
static uint32_t last_ts_ms = 0;
static uint8_t  have_loc   = 0; /* gates own DATA to RSU */
static int32_t  loc_x_dm = 0, loc_y_dm = 0; /* current pos (dm) */
static int32_t  prev_x_dm = 0, prev_y_dm = 0; /* previous pos (dm) */
static uint8_t  have_pos = 0;
static uint32_t t_last_move_ms = 0; /* last time we observed motion */
static uint16_t v_dmps = 0; /* speed (dm/s) */
static uint8_t  dir8   = 0; /* 0..7 (E,NE,N,NW,W,SW,S,SE) */
static uint8_t  ef_active = 0; /* legacy local EF flag (own) */

/* thresholds */
#define STOP_THRESH_DM 1   /* dx+dy <= 1 dm -> consider still */
#define STOP_MIN_MS    2000/* stand >= 2 s -> Emergency Braking */

/* ------------ Timers ------------------------------------------- */
static struct etimer t_query;
static struct etimer t_gossip;

/* ------------ EF LED state ------------------------------------- */
static uint8_t ef_led_on = 0;

/* ================================================================
 * Helper: EF LED state ON/OFF (no blinking).
 * ----------------------------------------------------------------
 * What: Turn red LED ON for EF!=FINISH, OFF for FINISH.
 * Methods: leds_on/off(LEDS_RED).
 * Creates: none.
 */
static void
ef_led_set(uint8_t on)
{
  ef_led_on = on ? 1 : 0;
  if(ef_led_on) leds_on(LEDS_RED);
  else leds_off(LEDS_RED);
}

/* ================================================================
 * NEW: EF registry (keep multiple active emergencies by origin)
 * ----------------------------------------------------------------
 * What:
 *  - Maintain a small table of active EF {origin_id -> ef_code}.
 *  - Provide add/update/remove and "last known" accessor.
 *  - Drive LED from the registry (ON if non-empty, else OFF).
 * Methods:
 *  - Linear scans; tiny fixed-size table (no dynamic memory).
 * Creates:
 *  - ef_tab[], ef_tick monotonic counter for "last" selection.
 */
#define EF_TAB_MAX 8

typedef struct {
  uint8_t  in_use;     /* 1 if slot used */
  uint32_t origin;     /* initiator OriginID */
  uint8_t  code;       /* WSAN_EF_* (not 0, not 255) */
  uint32_t tick;       /* updated order marker */
} ef_entry_t;

static ef_entry_t ef_tab[EF_TAB_MAX];
static uint32_t   ef_tick = 0;

/* Count active EF entries */
static uint8_t
ef_count_active(void)
{
  uint8_t n = 0;
  for(uint8_t i = 0; i < EF_TAB_MAX; ++i) n += (ef_tab[i].in_use != 0);
  return n;
}

/* Set LED according to registry non-emptiness (safety net) */
static void
ef_led_refresh(void)
{
  ef_led_set(ef_count_active() ? 1 : 0);
}

/* Find slot by origin */
static int8_t
ef_find(uint32_t origin)
{
  for(uint8_t i = 0; i < EF_TAB_MAX; ++i)
    if(ef_tab[i].in_use && ef_tab[i].origin == origin) return (int8_t)i;
  return -1;
}

/* Add new or update existing EF entry (code must be !=0 && !=255) */
static void
ef_add_or_update(uint32_t origin, uint8_t code)
{
  if(code == WSAN_EF_NONE || code == WSAN_EF_FINISH) return;
  int8_t idx = ef_find(origin);
  if(idx >= 0) {
    ef_tab[idx].code = code;
    ef_tab[idx].tick = ++ef_tick;
    return;
  }
  /* insert into first free slot; if none, replace oldest */
  int8_t free_i = -1;
  uint32_t min_tick = 0xFFFFFFFFu;
  int8_t   oldest_i = -1;
  for(uint8_t i = 0; i < EF_TAB_MAX; ++i) {
    if(!ef_tab[i].in_use && free_i < 0) free_i = (int8_t)i;
    if(ef_tab[i].in_use && ef_tab[i].tick < min_tick) {
      min_tick = ef_tab[i].tick; oldest_i = (int8_t)i;
    }
  }
  int8_t put = (free_i >= 0) ? free_i : oldest_i;
  ef_tab[put].in_use = 1;
  ef_tab[put].origin = origin;
  ef_tab[put].code   = code;
  ef_tab[put].tick   = ++ef_tick;
}

/* Remove EF entry by origin (FINISH semantics) */
static void
ef_remove(uint32_t origin)
{
  int8_t idx = ef_find(origin);
  if(idx >= 0) ef_tab[idx].in_use = 0;
}

/* Get "last known" EF (by tick). Returns 1 if exists, else 0. */
static uint8_t
ef_get_last(uint32_t *out_origin, uint8_t *out_code)
{
  int8_t best = -1;
  uint32_t best_tick = 0;
  for(uint8_t i = 0; i < EF_TAB_MAX; ++i) {
    if(ef_tab[i].in_use && ef_tab[i].tick >= best_tick) {
      best_tick = ef_tab[i].tick; best = (int8_t)i;
    }
  }
  if(best < 0) return 0;
  if(out_origin) *out_origin = ef_tab[best].origin;
  if(out_code)   *out_code   = ef_tab[best].code;
  return 1;
}

/* ================================================================
 * NEW: Is our own EF active?
 * ----------------------------------------------------------------
 * What: Return 1 if there is an EF entry with origin == self.
 * Methods: Scan ef_tab[]; compare origin_id.
 * Creates: none.
 */
static uint8_t
ef_self_active(void)
{
  for(uint8_t i = 0; i < EF_TAB_MAX; ++i) {
    if(ef_tab[i].in_use && ef_tab[i].origin == origin_id) return 1;
  }
  return 0;
}

/* ================================================================
 * Helper: UART1 line print with '\n' (for REQ_LOC).
 * ----------------------------------------------------------------
 * What: Send a zero-terminated string to UART1 followed by LF.
 * Methods: uart1_writeb().
 * Creates: none.
 */
static void
uart1_puts_ln(const char *s)
{
  if(!s) return;
  while(*s) uart1_writeb((uint8_t)*s++);
  uart1_writeb('\n');
}

/* ================================================================
 * Helper: int hypot approximation dx + dy/2 (no sqrt).
 * ----------------------------------------------------------------
 * What: Fast magnitude approximation without float/sqrt.
 * Methods: integer ops only.
 * Creates: none.
 */
static uint32_t
ihyp_dm(uint32_t adx, uint32_t ady)
{
  /* simple and cheap upper-boundish approximation */
  return adx + (ady >> 1);
}

/* ================================================================
 * Helper: map dx,dy to 8 directions (E,NE,N,NW,W,SW,S,SE).
 * ----------------------------------------------------------------
 * What: Discrete direction index 0..7 from integer dx,dy.
 * Methods: compare magnitudes; no trig.
 * Creates: none.
 */
static uint8_t
dir8_from_dxdy(int32_t dx, int32_t dy)
{
  /* 0:E 1:NE 2:N 3:NW 4:W 5:SW 6:S 7:SE */
  int32_t ax = dx >= 0 ? dx : -dx;
  int32_t ay = dy >= 0 ? dy : -dy;
  if(ax >= (ay<<1)) {
    return (dx >= 0) ? 0 : 4; /* E or W */
  } else if(ay >= (ax<<1)) {
    return (dy >= 0) ? 2 : 6; /* N or S */
  } else {
    if(dx >= 0 && dy >= 0) return 1; /* NE */
    if(dx < 0 && dy >= 0)  return 3; /* NW */
    if(dx < 0 && dy < 0)   return 5; /* SW */
    return 7; /* SE */
  }
}

/* ================================================================
 * Helper: pack metrics (x,y,v,dir) into small payload.
 * ----------------------------------------------------------------
 * Layout (7 bytes total):
 *  0..1: x_dm (int16, little-endian)
 *  2..3: y_dm (int16, little-endian)
 *  4..5: v_dmps (uint16, little-endian)
 *  6:    dir8 (0..7)
 * Creates: 7-byte array in 'out'.
 */
static void
pack_metrics(uint8_t out[7])
{
  int16_t  x = (int16_t)loc_x_dm;
  int16_t  y = (int16_t)loc_y_dm;
  uint16_t v = v_dmps;
  out[0] = (uint8_t)(x & 0xFF);
  out[1] = (uint8_t)((x >> 8) & 0xFF);
  out[2] = (uint8_t)(y & 0xFF);
  out[3] = (uint8_t)((y >> 8) & 0xFF);
  out[4] = (uint8_t)(v & 0xFF);
  out[5] = (uint8_t)((v >> 8) & 0xFF);
  out[6] = (uint8_t)dir8;
}

/* ================================================================
 * Helper: send EMERGENCY with metrics (or FINISH=255).
 * ----------------------------------------------------------------
 * What: Build Mbl/EMERGENCY with pl[4]=code and metrics after it.
 * Methods: wsan_pack(); fan-out; store to carry; optional flush.
 * Creates: EF frame in carry; uses ef_fanout_ctx.
 */
typedef struct {
  uint8_t buf[WSAN_MAX_FRAME];
  uint16_t len;
  uint8_t remaining;
  struct simple_udp_connection *udp;
  struct ctimer timer;
  uip_ipaddr_t maddr;
} fanout_ctx_t;

static void fanout_cb(void *ptr); /* fwd */
static fanout_ctx_t ef_fanout_ctx;
static fanout_ctx_t data_fanout_ctx;

static void
start_broadcast_fanout(fanout_ctx_t *ctx,
                       struct simple_udp_connection *udp,
                       const uint8_t *frame, uint16_t len,
                       uint8_t fanout)
{
  uip_create_linklocal_allnodes_mcast(&ctx->maddr);
  ctx->udp = udp;
  ctx->len = len;
  memcpy(ctx->buf, frame, len);
  simple_udp_sendto(ctx->udp, ctx->buf, ctx->len, &ctx->maddr);
  if(fanout <= 1) { ctx->remaining = 0; return; }
  ctx->remaining = (uint8_t)(fanout - 1);
  {
    uint16_t d = wsan_rand_jitter_ms();
    ctimer_set(&ctx->timer,
      (clock_time_t)(d * CLOCK_SECOND / 1000),
      fanout_cb, ctx);
  }
}

static void
fanout_cb(void *ptr)
{
  fanout_ctx_t *ctx = (fanout_ctx_t*)ptr;
  if(ctx->remaining == 0) return;
  simple_udp_sendto(ctx->udp, ctx->buf, ctx->len, &ctx->maddr);
  ctx->remaining--;
  if(ctx->remaining) {
    uint16_t d = wsan_rand_jitter_ms();
    ctimer_set(&ctx->timer,
      (clock_time_t)(d * CLOCK_SECOND / 1000),
      fanout_cb, ctx);
  }
}

/* forward decl from below for carry flush */
static void start_carry_flush(void);

static void
send_emergency(uint8_t ef_code)
{
  wsan_fields_t f; memset(&f, 0, sizeof(f));
  f.marker[0]='M'; f.marker[1]='b'; f.marker[2]='l';
  f.origin_id = origin_id;
  f.msg_type  = WSAN_MSG_EMERGENCY;
  f.msg_id    = wsan_make_msgid(&seq16);
  f.target_id = WSAN_TGT_BROADCAST;

  /* DEBUG: announce EF start/finish at the moment of issuing */
  if(ef_code != WSAN_EF_FINISH) {
    printf("Emergency Issued: id=%lu code=%u\n",
           (unsigned long)origin_id, (unsigned)ef_code);
  } else {
    printf("Emergency Finished: id=%lu\n", (unsigned long)origin_id);
  }

  /* payload: 'E','F','B','R', code, metrics[7] */
  uint8_t pl[5 + 7] = { 'E','F','B','R', ef_code };
  pack_metrics(&pl[5]);
  f.payload     = pl;
  f.payload_len = sizeof(pl);
  f.ts_ms       = last_ts_ms;

  uint8_t  buf[WSAN_MAX_FRAME];
  uint16_t out_len = 0;
  if(!wsan_pack(&f, buf, sizeof(buf), &out_len)) return;

  start_broadcast_fanout(&ef_fanout_ctx, &udp, buf, out_len,
                         WSAN_FANOUT_EF);

  /* keep in carry for later RSU upload */
  if(!wsan_carry_exists(&carry, f.origin_id, f.msg_id))
    wsan_carry_put(&carry, f.origin_id, f.msg_id, buf, out_len);

  if(have_rsu && carry.count > 0) start_carry_flush();
}

/* ================================================================
 * Helper: parse LOC "LOC id x_dm y_dm ts_ms" + motion detection.
 * ----------------------------------------------------------------
 * What: Update last_ts_ms, position, speed, dir; detect stop/start
 *       and send EF / EF_FINISH accordingly.
 * Methods: integer arithmetic; Manhattan thresholds; no float.
 * Creates: updates loc_x_dm, loc_y_dm, v_dmps, dir8, ef_active.
 */
static uint8_t
parse_loc_line(const char *line)
{
  if(!line) return 0;
  while(*line == ' ') line++;
  if(line[0]!='L' || line[1]!='O' || line[2]!='C') return 0;

  const char *p = line + 3;
  while(*p == ' ') p++;

  /* Parse: id x_dm y_dm ts_ms (space-separated) */
  long    parsed_id = strtol(p, (char**)&p, 10); while(*p == ' ') p++;
  int32_t xdm       = (int32_t)strtol(p, (char**)&p, 10); while(*p == ' ') p++;
  int32_t ydm       = (int32_t)strtol(p, (char**)&p, 10); while(*p == ' ') p++;
  uint32_t ts       = (uint32_t)strtoul(p, (char**)&p, 10);

  /* DEBUG: show parsed LOC */
  /*printf("LOC_RX: self=%lu parsed_id=%ld x=%ld y=%ld ts=%lu\n",
         (unsigned long)origin_id, parsed_id,
         (long)xdm, (long)ydm, (unsigned long)ts);*/

  /* Update timing: use previous ts snapshot for dt_ms */
  uint32_t prev_ts_snapshot = last_ts_ms;
  last_ts_ms = ts; have_loc = 1;

  if(!have_pos){
    prev_x_dm = loc_x_dm = xdm;
    prev_y_dm = loc_y_dm = ydm;
    t_last_move_ms = ts;
    v_dmps = 0; dir8 = 0;
    have_pos = 1;
    return 1;
  }

  /* compute delta */
  int32_t  dx  = xdm - loc_x_dm;
  int32_t  dy  = ydm - loc_y_dm;
  uint32_t adx = (dx >= 0) ? (uint32_t)dx : (uint32_t)(-dx);
  uint32_t ady = (dy >= 0) ? (uint32_t)dy : (uint32_t)(-dy);

  prev_x_dm = loc_x_dm; prev_y_dm = loc_y_dm;
  loc_x_dm = xdm;       loc_y_dm = ydm;

  /* direction update */
  dir8 = dir8_from_dxdy(dx, dy);

  /* speed (dm/s) from approx magnitude over dt_ms */
  uint32_t dt_ms = (ts >= prev_ts_snapshot) ?
                   (ts - prev_ts_snapshot) : 0;
  if(dt_ms == 0) dt_ms = 1;
  uint32_t mag = ihyp_dm(adx, ady); /* dm (approx) */
  v_dmps = (uint16_t)((mag * 1000U) / dt_ms);

  /* stop / start detector */
  uint32_t man = adx + ady;

  if(man > STOP_THRESH_DM) {
    /* moved -> reset still timer; finish EF if active */
    t_last_move_ms = ts;
    if(ef_active){
      send_emergency(WSAN_EF_FINISH); /* 255 */
      ef_active = 0;
      /* LED must go OFF on FINISH */
      ef_led_set(0);
      /* also clean registry, in case own entry exists */
      ef_remove(origin_id);
      ef_led_refresh();
    }
  } else {
    /* still */
    if(!ef_active){
      uint32_t stood = (ts >= t_last_move_ms) ?
                       (ts - t_last_move_ms) : 0;
      if(stood >= STOP_MIN_MS){

        /* First guard: if our own EF already active -> no duplicates */
        if(ef_self_active()) {
          /* optional debug */
          /* printf("EF_SKIP_SELF_DUP: id=%lu\n",
                 (unsigned long)origin_id); */
        } else {
          /* Second guard: if any foreign EF active -> suppress */
          uint8_t foreign = 0;
          for(uint8_t i = 0; i < EF_TAB_MAX; ++i) {
            if(ef_tab[i].in_use && ef_tab[i].origin != origin_id) {
              foreign = 1; break;
            }
          }
          if(foreign) {
            printf("EF_SUPPRESSED: id=%lu (foreign EF active)\n",
                   (unsigned long)origin_id);
          } else {
            /* declare Emergency Braking (code 1) */
            send_emergency(1);
            ef_active = 1;
            ef_led_set(1);
            /* add self-origin EF into registry */
            ef_add_or_update(origin_id, 1);
            ef_led_refresh();
          }
        }

      }
    }
  }
  return 1;
}

/* ================================================================
 * carry_put_logged: store frame into carry (ring with eviction)
 * ----------------------------------------------------------------
 * What: Put serialized frame, log eviction if ring is full.
 * Methods: wsan_carry_put(); printf for diagnostics (optional).
 * Creates: updates carry_evicted counter.
 */
static void
carry_put_logged(uint32_t origin, uint32_t msg,
                 const uint8_t *buf, uint16_t len)
{
  if(carry.count >= WSAN_CACHE_MAX_ITEMS) {
    carry_evicted++;
#if WSAN_DIAG
    printf("%lu,CARRY_EVICT,evicted=%lu\n",
      (unsigned long)clock_seconds(),
      (unsigned long)carry_evicted);
#endif
  }
  wsan_carry_put(&carry, origin, msg, buf, len);
#if WSAN_DIAG
  printf("%lu,CARRY_PUT,from=%lu,msg=%lu,count=%u\n",
    (unsigned long)clock_seconds(),
    (unsigned long)origin,
    (unsigned long)msg,
    (unsigned)carry.count);
#endif
}

/* ================================================================
 * Unicast with retries (non-blocking via ctimer)
 * ---------------------------------------------------------------- */
typedef struct {
  uint8_t buf[WSAN_MAX_FRAME];
  uint16_t len;
  uint8_t remaining;
  struct simple_udp_connection *udp;
  struct ctimer timer;
  uip_ipaddr_t dest;
} unicast_retry_ctx_t;

static unicast_retry_ctx_t uctx;

static void
unicast_retry_cb(void *ptr)
{
  unicast_retry_ctx_t *c = (unicast_retry_ctx_t*)ptr;
  if(c->remaining == 0) return;
  simple_udp_sendto(c->udp, c->buf, c->len, &c->dest);
  c->remaining--;
  if(c->remaining) {
    uint16_t d = wsan_rand_jitter_ms();
    ctimer_set(&c->timer,
      (clock_time_t)(d * CLOCK_SECOND / 1000),
      unicast_retry_cb, c);
  }
}

static void
start_unicast_with_retries(struct simple_udp_connection *udp,
                           const uip_ipaddr_t *dest,
                           const uint8_t *frame,
                           uint16_t len,
                           uint8_t retries_total)
{
  uctx.udp = udp;
  uctx.len = len;
  memcpy(uctx.buf, frame, len);
  uctx.dest = *dest;

  simple_udp_sendto(uctx.udp, uctx.buf, uctx.len, &uctx.dest);
  if(retries_total <= 1) { uctx.remaining = 0; return; }

  uctx.remaining = (uint8_t)(retries_total - 1);
  {
    uint16_t d = wsan_rand_jitter_ms();
    ctimer_set(&uctx.timer,
      (clock_time_t)(d * CLOCK_SECOND / 1000),
      unicast_retry_cb, &uctx);
  }
}

/* ================================================================
 * Build & send: QUERY (always), DATA (own), EF rebroadcast
 * ---------------------------------------------------------------- */

/* -- send_query(): broadcast WSAN/QUERY with EF status ----------- */
static void
send_query(void)
{
  wsan_fields_t f; memset(&f, 0, sizeof(f));
  f.marker[0]='M'; f.marker[1]='b'; f.marker[2]='l';
  f.origin_id = origin_id;
  f.msg_type  = WSAN_MSG_QUERY;
  f.msg_id    = wsan_make_msgid(&seq16);
  f.target_id = WSAN_TGT_BROADCAST;

  /* attach EF_flag(1) + EF_origin_id(4, BE) = 5 bytes */
  uint8_t qpl[5];
  uint32_t ef_orig = 0; uint8_t ef_flag = 0;
  if(!ef_get_last(&ef_orig, &ef_flag)) { ef_orig = 0; ef_flag = 0; }
  qpl[0] = ef_flag;
  wsan_u32_be_write(&qpl[1], ef_orig);

  f.payload     = qpl;
  f.payload_len = sizeof(qpl);
  f.ts_ms       = last_ts_ms;

  uint8_t  buf[WSAN_MAX_FRAME];
  uint16_t out_len = 0;
  if(!wsan_pack(&f, buf, sizeof(buf), &out_len)) return;

  fanout_ctx_t tmp;
  start_broadcast_fanout(&tmp, &udp, buf, out_len, 1);

#if WSAN_DIAG
  printf("%lu,QUERY,ef=%u,ef_orig=%lu\n",
         (unsigned long)clock_seconds(),
         (unsigned)ef_flag, (unsigned long)ef_orig);
#endif
}

/* -- send_data_unicast_to_rsu(): own data with metrics ----------- */
static void
send_data_unicast_to_rsu(void)
{
  if(!have_rsu || !have_loc) return;

  uint8_t pl[7]; pack_metrics(pl);

  wsan_fields_t f; memset(&f, 0, sizeof(f));
  f.marker[0]='M'; f.marker[1]='b'; f.marker[2]='l';
  f.origin_id = origin_id;
  f.msg_type  = WSAN_MSG_DATA;
  f.msg_id    = wsan_make_msgid(&seq16);
  f.target_id = rsu_id;
  f.payload     = pl;
  f.payload_len = sizeof(pl);
  f.ts_ms       = last_ts_ms;

  uint8_t  buf[WSAN_MAX_FRAME];
  uint16_t out_len = 0;
  if(!wsan_pack(&f, buf, sizeof(buf), &out_len)) return;

  start_unicast_with_retries(&udp, &rsu_ip, buf, out_len,
                             WSAN_UNICAST_RETRIES);
}

/* ================================================================
 * Carry flushing to RSU (paced)
 * ---------------------------------------------------------------- */
typedef struct {
  struct simple_udp_connection *udp;
  uip_ipaddr_t dest;
  uint16_t idx, left;
  struct ctimer timer;
} carry_flush_ctx_t;

static carry_flush_ctx_t cflush;

/* -- carry_flush_cb(): send next stored frame -------------------- */
static void
carry_flush_cb(void *ptr)
{
  carry_flush_ctx_t *cf = (carry_flush_ctx_t*)ptr;
  while(cf->left) {
    uint16_t i = cf->idx % WSAN_CACHE_MAX_ITEMS;
    wsan_frame_slot_t *s = &carry.slots[i];
    cf->idx++; cf->left--;
    if(s->len) {
#if WSAN_DIAG
      printf("%lu,CARRY_TX,idx=%u,left=%u,bytes=%u\n",
        (unsigned long)clock_seconds(),
        (unsigned)(cf->idx - 1),
        (unsigned)cf->left,
        (unsigned)s->len);
#endif
      simple_udp_sendto(cf->udp, s->buf, s->len, &cf->dest);
      ctimer_set(&cf->timer,
        (clock_time_t)(5 * CLOCK_SECOND / 1000),
        carry_flush_cb, cf);
      return;
    }
  }
  wsan_carry_init(&carry);
}

/* -- start_carry_flush(): kick flushing of all frames ------------ */
static void
start_carry_flush(void)
{
  if(!have_rsu || carry.count == 0) return;
  cflush.udp  = &udp;
  cflush.dest = rsu_ip;
  cflush.idx  = 0;
  cflush.left = carry.count;
  carry_flush_cb(&cflush);
}

/* ================================================================
 * UDP RX callback
 * ----------------------------------------------------------------
 * What: Parse WSAN frames, dedup, learn RSU on ACK.
 *       Always accept DATA/EF, store to carry.
 *       If RSU visible and carry non-empty — start flush now.
 *       Parse QUERY from mobiles to track EF via payload.
 */
static void
udp_rx_cb(struct simple_udp_connection *c,
          const uip_ipaddr_t *sender_addr,
          uint16_t sender_port,
          const uip_ipaddr_t *receiver_addr,
          uint16_t receiver_port,
          const uint8_t *data, uint16_t datalen)
{
  (void)c; (void)sender_port; (void)receiver_addr; (void)receiver_port;

  wsan_fields_t h; const uint8_t *pl; uint16_t pl_len;
  if(!wsan_parse(data, datalen, &h, &pl, &pl_len)) return;

  /* --- DEDUP front: drop duplicates for all types, incl. EF ----- */
  if(wsan_dedup_has(&dedup, h.origin_id, h.msg_id)) return;
  wsan_dedup_put(&dedup, h.origin_id, h.msg_id);

  /* --- RSU ACK: mark presence and refresh epoch ----------------- */
  if(h.marker[0]=='R' && h.marker[1]=='S' && h.marker[2]=='U') {
    if(h.msg_type == WSAN_MSG_ACK) {
      rsu_ip  = *sender_addr;
      rsu_id  = h.origin_id;
      have_rsu = 1;
      ack_epoch_last = probe_epoch;
#if WSAN_DIAG
      printf("%lu,STAT,HAVE_RSU=1,rsu_id=%lu\n",
        (unsigned long)clock_seconds(),
        (unsigned long)rsu_id);
#endif
      ctimer_stop(&uctx.timer); uctx.remaining = 0;
      if(carry.count > 0) start_carry_flush();
    }
    return;
  }

  /* --- Only mobile frames ("Mbl") beyond this point -------------- */
  if(h.marker[0]!='M' || h.marker[1]!='b' || h.marker[2]!='l') return;

  /* Parse QUERY from other mobiles to absorb EF state */
  if(h.msg_type == WSAN_MSG_QUERY) {
    if(pl && pl_len >= 5) {
      uint8_t  ef_flag = pl[0];
      uint32_t ef_orig = wsan_u32_be_read(&pl[1]);
      if(ef_flag == WSAN_EF_NONE) {
        /* no change */
      } else if(ef_flag == WSAN_EF_FINISH) {
        ef_remove(ef_orig);
        ef_led_refresh();
      } else {
        ef_add_or_update(ef_orig, ef_flag);
        ef_led_refresh();
      }
#if WSAN_DIAG
      printf("%lu,QUERY_RX,from=%lu,ef=%u,ef_orig=%lu\n",
        (unsigned long)clock_seconds(),
        (unsigned long)h.origin_id,
        (unsigned)ef_flag, (unsigned long)ef_orig);
#endif
    }
    return;
  }

  if(h.msg_type == WSAN_MSG_DATA) {
#if WSAN_DIAG
    printf("%lu,DATA_RX,from=%lu,len=%u\n",
      (unsigned long)clock_seconds(),
      (unsigned long)h.origin_id, (unsigned)datalen);
#endif
    if(!wsan_carry_exists(&carry, h.origin_id, h.msg_id))
      carry_put_logged(h.origin_id, h.msg_id, data, datalen);
    if(have_rsu && carry.count > 0) start_carry_flush();
    return;
  }

  if(h.msg_type == WSAN_MSG_EMERGENCY) {
    /* EF LED reflect remote EF code (legacy behavior) */
    if(pl && pl_len >= 5) {
      uint8_t efc = pl[4];
      ef_led_set(efc != WSAN_EF_FINISH);

      /* update EF registry to handle multiple EF correctly */
      if(efc == WSAN_EF_FINISH) ef_remove(h.origin_id);
      else                      ef_add_or_update(h.origin_id, efc);

      /* ensure LED matches "any EF active" semantics */
      ef_led_refresh();
    }
    if(!wsan_carry_exists(&carry, h.origin_id, h.msg_id))
      carry_put_logged(h.origin_id, h.msg_id, data, datalen);

    /* re-broadcast EF/Finish with fan-out */
    start_broadcast_fanout(&ef_fanout_ctx, &udp, data, datalen,
                           WSAN_FANOUT_EF);
    if(have_rsu && carry.count > 0) start_carry_flush();
    return;
  }
}

/* ================================================================
 * Process thread
 * ----------------------------------------------------------------
 * What: Register UDP, init serial line & timers, main loop.
 * Methods: send QUERY each tick; apply 3-misses rule; flush carry
 *          when RSU visible; own DATA to RSU if LOC; periodic
 *          REQ_LOC.
 * Creates: none.
 */
PROCESS(mobile_process, "WSAN Mobile");
AUTOSTART_PROCESSES(&mobile_process);

PROCESS_THREAD(mobile_process, ev, data)
{
  PROCESS_BEGIN();

  simple_udp_register(&udp, WSAN_UDP_PORT, NULL,
                      WSAN_UDP_PORT, udp_rx_cb);

  serial_line_init();
  uart1_set_input(serial_line_input_byte);

  node_id8  = linkaddr_node_addr.u8[7];
  origin_id = (uint32_t)node_id8;
  random_init(origin_id);

  wsan_dedup_init(&dedup);
  wsan_carry_init(&carry);

  etimer_set(&t_query,  WSAN_T_QUERY_MS  * CLOCK_SECOND / 1000);
  etimer_set(&t_gossip, WSAN_T_GOSSIP_MS * CLOCK_SECOND / 1000);

  while(1) {
    PROCESS_YIELD();

    /* Serial LOC lines (from Script Engine) */
    if(ev == serial_line_event_message && data) {
      (void)parse_loc_line((const char*)data);
    }

    /* --- Periodic QUERY tick (+ 3-misses rule + REQ_LOC) -------- */
    if(etimer_expired(&t_query)) {
      probe_epoch++;
      send_query();

      /* also request position from Script Engine */
      uart1_puts_ln("REQ_LOC");

      if(have_rsu) {
        uint32_t delta = probe_epoch - ack_epoch_last;
        if(delta >= 3) {
          have_rsu = 0;
          ctimer_stop(&uctx.timer); uctx.remaining = 0;
#if WSAN_DIAG
          printf("%lu,STAT,HAVE_RSU=0\n",
            (unsigned long)clock_seconds());
#endif
        }
      }
      etimer_reset(&t_query);
    }

    /* --- If RSU is visible and carry is non-empty — flush -------- */
    if(have_rsu && carry.count > 0) start_carry_flush();

    /* --- Optional: own DATA to RSU when LOC is available -------- */
    if(have_rsu && have_loc) {
      send_data_unicast_to_rsu();
    }

    /* --- Gossip own DATA when RSU is not visible (with metrics) -- */
    if(!have_rsu && etimer_expired(&t_gossip)) {
      uint8_t pl[7]; pack_metrics(pl);

      wsan_fields_t f; memset(&f, 0, sizeof(f));
      f.marker[0]='M'; f.marker[1]='b'; f.marker[2]='l';
      f.origin_id = origin_id;
      f.msg_type  = WSAN_MSG_DATA;
      f.msg_id    = wsan_make_msgid(&seq16);
      f.target_id = WSAN_TGT_BROADCAST;
      f.payload     = pl;
      f.payload_len = sizeof(pl);
      f.ts_ms       = last_ts_ms;

      uint8_t  buf[WSAN_MAX_FRAME];
      uint16_t out_len = 0;
      if(wsan_pack(&f, buf, sizeof(buf), &out_len)) {
        start_broadcast_fanout(&data_fanout_ctx, &udp,
                               buf, out_len, WSAN_FANOUT_DATA);
      }
      etimer_reset(&t_gossip);
    }
  }

  PROCESS_END();
}