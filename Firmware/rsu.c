/*
 * RSU mote (WSAN):
 * - On QUERY/DATA/EMERGENCY from Mobile: ACK (unicast), EF rebroadcast once.
 * - RSU does NOT create/finish emergencies.
 * - Dedup by (OriginID,MsgID).
 * - serial-line is bound to UART0, echo with printf (UART0).
 * - Type "STOP 3"/"GO 3" in RSU Serial port -> RSU prints the same line.
 */

#include "contiki.h"
#include "net/ipv6/simple-udp.h"
#include "os/sys/ctimer.h"
#include "os/sys/etimer.h"
#include "dev/serial-line.h"
#include "dev/uart1.h"   /* <-- bind serial_line to UART0 */
#include "random.h"
#include <stdio.h>
#include <string.h>

#include "wsan_common.h"
#include "wsan_protocol.h"

/* ---------- Metric logging control ---------- */
#ifndef WSAN_METRIC_LOG_ENABLED
#define WSAN_METRIC_LOG_ENABLED 1
#endif
/* -------------------------------------------- */

static struct simple_udp_connection udp;
static uint8_t  node_id8;
static uint32_t origin_id;   /* RSU OriginID */
static uint16_t seq16 = 0;
static wsan_dedup_t dedup;

/* ---------- METRIC: mote output logging ---------- */
#if WSAN_METRIC_LOG_ENABLED
static void
rsu_log_location(uint32_t mote_id,
                 int32_t x_dm,
                 int32_t y_dm,
                 uint32_t ts_ms)
{
    uint32_t s = clock_time() / CLOCK_SECOND;
    printf(
        "[%02lu:%02lu:%02lu] [Coord%lu] [SENSOR] "
        "mote=%lu data=x_dm=%ld y_dm=%ld ts_ms=%lu rssi=0\n",
        s / 3600,
        (s % 3600) / 60,
        s % 60,
        (unsigned long)origin_id,
        (unsigned long)mote_id,
        (long)x_dm,
        (long)y_dm,
        (unsigned long)ts_ms
    );
}
#endif
/* ------------------------------------------------ */

/* ---------- Fan-out (EF) ---------- */
typedef struct {
  uint8_t  buf[WSAN_MAX_FRAME];
  uint16_t len;
  uint8_t  remaining;
  struct simple_udp_connection *udp;
  struct ctimer timer;
  uip_ipaddr_t maddr;
} rsu_fanout_ctx_t;

static rsu_fanout_ctx_t rsu_ef_ctx;

static void rsu_fanout_cb(void *ptr)
{
  rsu_fanout_ctx_t *ctx = (rsu_fanout_ctx_t*)ptr;
  if(ctx->remaining == 0) return;
  simple_udp_sendto(ctx->udp, ctx->buf, ctx->len, &ctx->maddr);
  ctx->remaining--;
  if(ctx->remaining) {
    uint16_t d = wsan_rand_jitter_ms();
    ctimer_set(&ctx->timer,
               (clock_time_t)(d * CLOCK_SECOND / 1000),
               rsu_fanout_cb, ctx);
  }
}

static void rsu_start_fanout(rsu_fanout_ctx_t *ctx,
                             const uint8_t *frame, uint16_t len,
                             uint8_t fanout)
{
  uip_create_linklocal_allnodes_mcast(&ctx->maddr);
  ctx->udp = &udp;
  ctx->len = len;
  memcpy(ctx->buf, frame, len);

  /* first send immediately */
  simple_udp_sendto(ctx->udp, ctx->buf, ctx->len, &ctx->maddr);

  if(fanout <= 1) { ctx->remaining = 0; return; }
  ctx->remaining = (uint8_t)(fanout - 1);
  {
    uint16_t d = wsan_rand_jitter_ms();
    ctimer_set(&ctx->timer,
               (clock_time_t)(d * CLOCK_SECOND / 1000),
               rsu_fanout_cb, ctx);
  }
}

/* ---------- ACK helper ---------- */
static void send_ack_to(const uip_ipaddr_t *dst,
                        uint32_t target_id, uint32_t ts_ms)
{
  wsan_fields_t f;
  memset(&f, 0, sizeof(f));
  f.marker[0] = 'R'; f.marker[1] = 'S'; f.marker[2] = 'U';
  f.origin_id  = origin_id;
  f.msg_type   = WSAN_MSG_ACK;
  f.msg_id     = wsan_make_msgid(&seq16);
  f.target_id  = target_id;
  f.payload    = NULL;
  f.payload_len= 0;
  f.ts_ms      = ts_ms;

  uint8_t  buf[WSAN_MAX_FRAME];
  uint16_t out_len = 0;
  if(!wsan_pack(&f, buf, sizeof(buf), &out_len)) return;

  simple_udp_sendto(&udp, buf, out_len, dst);
}

/* ---------- UDP RX ---------- */
static void udp_rx_cb(struct simple_udp_connection *c,
                      const uip_ipaddr_t *sender_addr,
                      uint16_t sender_port,
                      const uip_ipaddr_t *receiver_addr,
                      uint16_t receiver_port,
                      const uint8_t *data, uint16_t datalen)
{
  (void)c; (void)sender_port; (void)receiver_addr; (void)receiver_port;

  wsan_fields_t h; const uint8_t *pl; uint16_t pl_len;
  if(!wsan_parse(data, datalen, &h, &pl, &pl_len)) return;

  /* Dedup */
  if(wsan_dedup_has(&dedup, h.origin_id, h.msg_id)) return;
  wsan_dedup_put(&dedup, h.origin_id, h.msg_id);

  /* Accept only from Mobiles ("Mbl") */
  if(h.marker[0]!='M' || h.marker[1]!='b' || h.marker[2]!='l') return;

  if(h.msg_type == WSAN_MSG_QUERY) {
    send_ack_to(sender_addr, h.origin_id, h.ts_ms);
    return;
  }
  if(h.msg_type == WSAN_MSG_DATA) {
    send_ack_to(sender_addr, h.origin_id, h.ts_ms);
    
#if WSAN_METRIC_LOG_ENABLED
        /* DATA contains x_dm,y_dm at payload[0..7] */
        printf("RSU logging 1:\n");
        if(pl && pl_len >= sizeof(wsan_data_payload_t)) {
            const wsan_data_payload_t *d =
                (const wsan_data_payload_t *)pl;

            rsu_log_location(h.origin_id,
                             d->x_dm,
                             d->y_dm,
                             h.ts_ms,
                             d->rssi_dbm);
        }
#endif

    return;
  }
  if(h.msg_type == WSAN_MSG_EMERGENCY) {
    send_ack_to(sender_addr, h.origin_id, h.ts_ms);
#if WSAN_METRIC_LOG_ENABLED
        printf("RSU logging 2:\n");
        if(pl && pl_len >= 
sizeof(wsan_emergency_payload_t)) {
        const wsan_emergency_payload_t *e =
            (const wsan_emergency_payload_t *)pl;

        rsu_log_location(h.origin_id,
                         e->x_dm,
                         e->y_dm,
                         h.ts_ms,
                         e->rssi_dbm);
    }
#endif

    rsu_start_fanout(&rsu_ef_ctx, data, datalen, WSAN_FANOUT_EF);
    return;
  }
}

/* ---------- RSU process ---------- */
PROCESS(rsu_process, "WSAN RSU");
AUTOSTART_PROCESSES(&rsu_process);

PROCESS_THREAD(rsu_process, ev, data)
{
  PROCESS_BEGIN();
  simple_udp_register(&udp, WSAN_UDP_PORT, NULL,
                      WSAN_UDP_PORT, udp_rx_cb);

  /* Serial line on UART0 (the panel "Serial port" in Cooja) */
  serial_line_init();
  uart1_set_input(serial_line_input_byte);

  node_id8  = linkaddr_node_addr.u8[7];
  origin_id = (uint32_t)node_id8;
  random_init(origin_id);

  wsan_dedup_init(&dedup);

  /* Event-driven RSU */
  while(1) {
    PROCESS_YIELD();

    /* Echo back any received line to UART0 (printf -> Serial port) */
    if(ev == serial_line_event_message && data) {
      const char *s = (const char*)data;
      printf("%s\n", s);
    }
  }

  PROCESS_END();
}