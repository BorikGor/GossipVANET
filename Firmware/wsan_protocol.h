#ifndef WSAN_PROTOCOL_H_
#define WSAN_PROTOCOL_H_

/*
 * WSAN protocol (v1.4-compatible) + compact inline utilities.
 * This header intentionally *replaces* the old wsan_helpers.h to
 * avoid duplication and ODR issues. Do not include wsan_helpers.h.
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "contiki.h"
#include "random.h"
#include "sys/clock.h"
#include "net/ipv6/simple-udp.h"

#include "wsan_common.h"

/* ------------------------------------------------------------------ */
/*  Checksum (Chk8)                                                    */
/* ------------------------------------------------------------------ */
/*
 * wsan_chk8()
 * What:     Compute LSB of sum of bytes in buffer.
 * Methods:  Linear pass, uint16 accumulator, LSB at the end.
 * Creates:  local uint16_t 's' accumulator.
 */
static inline uint8_t wsan_chk8(const uint8_t *buf, size_t len)
{
    uint16_t s = 0;
    for(size_t i = 0; i < len; ++i) s += buf[i];
    return (uint8_t)(s & 0xFF);
}

/* ------------------------------------------------------------------ */
/*  BE read/write (u32)                                                */
/* ------------------------------------------------------------------ */
/*
 * wsan_u32_be_write()
 * What:     Store 32-bit value in Big-Endian order into p[0..3].
 * Methods:  Shift-and-mask.
 * Creates:  none.
 */
static inline void wsan_u32_be_write(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)((v >> 24) & 0xFF);
    p[1] = (uint8_t)((v >> 16) & 0xFF);
    p[2] = (uint8_t)((v >> 8)  & 0xFF);
    p[3] = (uint8_t)((v)       & 0xFF);
}

/*
 * wsan_u32_be_read()
 * What:     Read Big-Endian 32-bit value from p[0..3].
 * Methods:  Shift-and-or.
 * Creates:  none.
 */
static inline uint32_t wsan_u32_be_read(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24)
         | ((uint32_t)p[1] << 16)
         | ((uint32_t)p[2] << 8)
         | ((uint32_t)p[3]);
}

/* ------------------------------------------------------------------ */
/*  Random jitter (0..WSAN_JITTER_MS)                                  */
/* ------------------------------------------------------------------ */
/*
 * wsan_rand_jitter_ms()
 * What:     Produce 0..WSAN_JITTER_MS pseudo-random jitter in ms.
 * Methods:  Scale 16-bit random_rand() to [0..JITTER].
 * Creates:  none.
 */
static inline uint16_t wsan_rand_jitter_ms(void)
{
    return (uint16_t)(((uint32_t)(random_rand() & 0xFFFFu) *
                       (uint32_t)WSAN_JITTER_MS) / 65535u);
}

/* ------------------------------------------------------------------ */
/*  Direction quantization (octants)                                   */
/* ------------------------------------------------------------------ */
/*
 * wsan_dir_from_delta()
 * What:     Map dx,dy (dm or any linear unit) to octant 0..7 or UNK.
 * Methods:  No floats/atan2; compare |dx| and |dy| with factor 2.
 * Creates:  local adx, ady, qx, qy.
 */
static inline uint8_t wsan_dir_from_delta(int32_t dx, int32_t dy)
{
    //if (dx == 0 && dy == 0) return WSAN_DIR_UNK;

    int32_t adx = dx < 0 ? -dx : dx;
    int32_t ady = dy < 0 ? -dy : dy;

    if (adx >= (ady << 1))
        return (dx >= 0) ? WSAN_DIR_E : WSAN_DIR_W;

    if (ady >= (adx << 1))
        return (dy >= 0) ? WSAN_DIR_N : WSAN_DIR_S;

    if (dx >= 0 && dy >= 0) return WSAN_DIR_NE;
    if (dx <  0 && dy >= 0) return WSAN_DIR_NW;
    if (dx <  0 && dy <  0) return WSAN_DIR_SW;
    return WSAN_DIR_SE;
}

/* ------------------------------------------------------------------ */
/*  Dedup (very small fixed-size ring)                                 */
/* ------------------------------------------------------------------ */
#ifndef WSAN_DEDUP_MAX
#define WSAN_DEDUP_MAX 64
#endif

typedef struct {
    uint32_t origin_id;
    uint32_t msg_id;
} wsan_key_t;

typedef struct {
    wsan_key_t items[WSAN_DEDUP_MAX];
    uint8_t    used;
    uint8_t    head; /* ring index for overwrite */
} wsan_dedup_t;

/*
 * wsan_dedup_init()
 * What:     Reset dedup ring.
 * Methods:  memset.
 * Creates:  none.
 */
static inline void wsan_dedup_init(wsan_dedup_t *d)
{
    memset(d, 0, sizeof(*d));
}

/*
 * wsan_dedup_has()
 * What:     Check if (origin,msg) exists in ring.
 * Methods:  linear scan up to 'used'.
 * Creates:  loop index idx.
 */
static inline uint8_t wsan_dedup_has(wsan_dedup_t *d,
                                     uint32_t origin,
                                     uint32_t msg)
{
    for(uint8_t i = 0; i < d->used; ++i) {
        uint8_t idx = (uint8_t)((i < WSAN_DEDUP_MAX) ? i
                                                    : (i % WSAN_DEDUP_MAX));
        if(d->items[idx].origin_id == origin &&
           d->items[idx].msg_id    == msg) return 1;
    }
    return 0;
}

/*
 * wsan_dedup_put()
 * What:     Insert (origin,msg) at head, overwrite when full.
 * Methods:  ring buffer with head++ mod size.
 * Creates:  none.
 */
static inline void wsan_dedup_put(wsan_dedup_t *d,
                                  uint32_t origin,
                                  uint32_t msg)
{
    d->items[d->head].origin_id = origin;
    d->items[d->head].msg_id    = msg;
    d->head = (uint8_t)((d->head + 1) % WSAN_DEDUP_MAX);
    if(d->used < WSAN_DEDUP_MAX) d->used++;
}

/* ------------------------------------------------------------------ */
/*  Carry store (Mobile)                                               */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t origin_id;
    uint32_t msg_id;
    uint16_t len;
    uint8_t  buf[WSAN_MAX_FRAME];
} wsan_frame_slot_t;

typedef struct {
    wsan_frame_slot_t slots[WSAN_CACHE_MAX_ITEMS];
    uint16_t          count;
    uint16_t          head; /* next write */
} wsan_carry_t;

/*
 * wsan_carry_init()
 * What:     Reset carry store.
 * Methods:  memset.
 * Creates:  none.
 */
static inline void wsan_carry_init(wsan_carry_t *c)
{
    memset(c, 0, sizeof(*c));
}

/*
 * wsan_carry_exists()
 * What:     Check if (origin,msg) present.
 * Methods:  linear scan up to 'count'.
 * Creates:  loop index idx.
 */
static inline uint8_t wsan_carry_exists(wsan_carry_t *c,
                                        uint32_t origin,
                                        uint32_t msg)
{
    for(uint16_t i = 0; i < c->count; ++i) {
        uint16_t idx = (uint16_t)((i < WSAN_CACHE_MAX_ITEMS) ? i
                         : (i % WSAN_CACHE_MAX_ITEMS));
        if(c->slots[idx].origin_id == origin &&
           c->slots[idx].msg_id    == msg) return 1;
    }
    return 0;
}

/*
 * wsan_carry_put()
 * What:     Store a serialized frame, count-limited ring (no TTL).
 * Methods:  overwrite at head; clamp len to WSAN_MAX_FRAME.
 * Creates:  local idx.
 */
static inline void wsan_carry_put(wsan_carry_t *c,
                                  uint32_t origin,
                                  uint32_t msg,
                                  const uint8_t *buf,
                                  uint16_t len)
{
    uint16_t idx = c->head;
    if(len > WSAN_MAX_FRAME) len = WSAN_MAX_FRAME;
    c->slots[idx].origin_id = origin;
    c->slots[idx].msg_id    = msg;
    c->slots[idx].len       = len;
    memcpy(c->slots[idx].buf, buf, len);
    c->head = (uint16_t)((c->head + 1) % WSAN_CACHE_MAX_ITEMS);
    if(c->count < WSAN_CACHE_MAX_ITEMS) c->count++;
}

/* ------------------------------------------------------------------ */
/*  MsgID helper                                                       */
/* ------------------------------------------------------------------ */
/*
 * wsan_make_msgid()
 * What:     Build MsgID = [HH][MM][SEQ16] (BE on the wire in pack()).
 * Methods:  Use clock_seconds(); reset seq if minute changed.
 * Creates:  static g_min_snapshot; local hh, mm, seq.
 */
static inline uint32_t wsan_make_msgid(uint16_t *p_seq16 /* in/out */)
{
    static uint8_t g_min_snapshot = 0;
    uint32_t secs = clock_seconds();
    uint8_t  hh   = (uint8_t)((secs / 3600) % 24);
    uint8_t  mm   = (uint8_t)((secs / 60)   % 60);

    if(mm != g_min_snapshot) { g_min_snapshot = mm; *p_seq16 = 0; }
    uint16_t seq = (*p_seq16)++;

    return ((uint32_t)hh << 24) | ((uint32_t)mm << 16) | seq;
}

/* ------------------------------------------------------------------ */
/*  WSAN wire format (pack/parse)                                      */
/* ------------------------------------------------------------------ */
/*
 * WSAN frame (v1.4) on the wire (BE multi-byte fields):
 * [ "WSAN"(4) ]
 * [ MsgLen(3) ] // [00][hi][lo], bytes from Marker .. CS inclusive
 * [ Marker(3) ] // "Mbl" or "RSU"
 * [ OriginID(4) ] // BE
 * [ MsgType(1) ]
 * [ MsgID(4) ] // BE
 * [ TargetID(4) ] // BE
 * [ Payload(..) ]
 * [ TimeStamp(4) ] // BE (ms from simulator/script)
 * [ CS(1) ] // LSB of sum from 'W' to end of TimeStamp
 */
typedef struct {
    char        marker[3];
    uint32_t    origin_id;
    uint8_t     msg_type;
    uint32_t    msg_id;
    uint32_t    target_id;
    uint16_t    payload_len;
    const uint8_t *payload;
    uint32_t    ts_ms;
} wsan_fields_t;

/* =========================================================================
 * Unified WSAN payload descriptions
 * -------------------------------------------------------------------------
 *  Describe the meaning/layout of payload bytes.
 * ========================================================================= */

/* DATA payload: location + motion metrics */
typedef struct {
    int16_t  x_dm;        /* position X, decimeters */
    int16_t  y_dm;        /* position Y, decimeters */
    uint16_t v_dmps;      /* speed, dm/s */
    uint8_t  dir8;        /* direction 0..7 */
    int8_t   rssi_dbm;    /* RSSI placeholder (0 in Cooja) */
} wsan_data_payload_t;

/* EMERGENCY payload: EF code + same metrics */
typedef struct {
    uint8_t  ef_code;     /* WSAN_EF_* */
    int16_t  x_dm;
    int16_t  y_dm;
    uint16_t v_dmps;
    uint8_t  dir8;
    int8_t   rssi_dbm;    /* RSSI placeholder */
} wsan_emergency_payload_t;

/*
 * wsan_pack()
 * What:     Serialize fields into 'out'; return total len in out_len.
 * Methods:  Fill header, write BE fields, compute body len and CS.
 * Creates:  local body_len, pre_cs_len, cs.
 */
static inline uint8_t wsan_pack(const wsan_fields_t *f,
                                uint8_t *out,
                                uint16_t cap,
                                uint16_t *out_len)
{
    if(!f || !out || cap < 28) return 0;

    /* "WSAN" */
    out[0] = 'W'; out[1] = 'S'; out[2] = 'A'; out[3] = 'N';

    /* MsgLen placeholder */
    out[4] = 0x00; out[5] = 0x00; out[6] = 0x00;

    /* Marker */
    out[7] = (uint8_t)f->marker[0];
    out[8] = (uint8_t)f->marker[1];
    out[9] = (uint8_t)f->marker[2];

    /* OriginID BE */
    wsan_u32_be_write(&out[10], f->origin_id);

    /* MsgType */
    out[14] = f->msg_type;

    /* MsgID BE */
    wsan_u32_be_write(&out[15], f->msg_id);

    /* TargetID BE */
    wsan_u32_be_write(&out[19], f->target_id);

    /* Payload */
    if(f->payload_len > (WSAN_MAX_FRAME - 28)) return 0;
    if(f->payload_len && f->payload) {
        memcpy(&out[23], f->payload, f->payload_len);
    }

    /* TimeStamp BE */
    wsan_u32_be_write(&out[23 + f->payload_len], f->ts_ms);

    /* Body length (Marker..CS inclusive) */
    uint16_t body_len = (uint16_t)(3 + 4 + 1 + 4 + 4
                             + f->payload_len + 4 + 1);

    /* Fill MsgLen hi/lo */
    out[5] = (uint8_t)((body_len >> 8) & 0xFF);
    out[6] = (uint8_t)( body_len       & 0xFF);

    /* CS over all bytes before CS */
    uint16_t pre_cs_len = (uint16_t)(4 + 3 + body_len - 1);
    uint8_t cs = wsan_chk8(out, pre_cs_len);
    out[23 + f->payload_len + 4] = cs;

    *out_len = (uint16_t)(pre_cs_len + 1);
    return 1;
}

/*
 * wsan_parse()
 * What:     Validate and parse WSAN frame from 'in'.
 * Methods:  Check signature/length/CS; expose payload view.
 * Creates:  local body_len, expect, cs_calc, cs_got, pay_len.
 */
static inline uint8_t wsan_parse(const uint8_t *in,
                                 uint16_t in_len,
                                 wsan_fields_t *f,
                                 const uint8_t **pl,
                                 uint16_t *pl_len)
{
    if(!in || in_len < 28 || !f || !pl || !pl_len) return 0;
    if(in[0] != 'W' || in[1] != 'S' || in[2] != 'A' || in[3] != 'N')
        return 0;

    uint16_t body_len = (uint16_t)(((in[5] << 8) | in[6]));
    uint16_t expect   = (uint16_t)(4 + 3 + body_len);
    if(in_len != expect) return 0;

    uint8_t cs_calc = wsan_chk8(in, (uint16_t)(expect - 1));
    uint8_t cs_got  = in[expect - 1];
    if(cs_calc != cs_got) return 0;

    f->marker[0]  = (char)in[7];
    f->marker[1]  = (char)in[8];
    f->marker[2]  = (char)in[9];
    f->origin_id  = wsan_u32_be_read(&in[10]);
    f->msg_type   = in[14];
    f->msg_id     = wsan_u32_be_read(&in[15]);
    f->target_id  = wsan_u32_be_read(&in[19]);

    uint16_t pay_len = (uint16_t)(body_len - (3+4+1+4+4+4+1));
    *pl      = &in[23];
    *pl_len  = pay_len;

    f->payload     = *pl;
    f->payload_len = *pl_len;
    f->ts_ms       = wsan_u32_be_read(&in[23 + pay_len]);

    return 1;
}

#endif /* WSAN_PROTOCOL_H_ */