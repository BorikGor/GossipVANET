#ifndef WSAN_COMMON_H_
#define WSAN_COMMON_H_
/*
 * WSAN common definitions (roles, message types, flags, tuning).
 * - Network byte order on the wire is Big-Endian (BE).
 * - This header contains *only* constants shared across modules.
 *
 * Radio / PHY references (for choosing sensible defaults):
 * - IEEE 802.15.4 @ 2.4 GHz = 250 kbps (TI CC2420).
 * - TelosB / Tmote Sky typical range ~125 m outdoor.
 */

/* ===== Roles (3-char marker in the header) ===== */
#define WSAN_MARK_MOBILE "Mbl" /* Mobile mote */
#define WSAN_MARK_RSU    "RSU" /* Road Side Unit */

/* ===== Message types (WSAN v1.4) ===== */
#define WSAN_MSG_QUERY     0
#define WSAN_MSG_ACK       1
#define WSAN_MSG_DATA      2
#define WSAN_MSG_EMERGENCY 4

/* ===== Emergency flags (EF) ===== */
#define WSAN_EF_NONE   0   /* no emergency */
#define WSAN_EF_BRAKE  1   /* sudden braking / crash (M2M-critical) */
#define WSAN_EF_OIL    2   /* oil temperature emergency (for RSU) */
#define WSAN_EF_BATT   3   /* battery emergency (for RSU) */
#define WSAN_EF_FINISH 255 /* finish / clear emergency (creator only) */

/* ===== UDP port ===== */
#define WSAN_UDP_PORT 8765

/* ===== Broadcast TargetID ===== */
#define WSAN_TGT_BROADCAST 0xFFFFFFFFu

/* ===== Frame sizes ===== */
#ifndef WSAN_MAX_FRAME
#define WSAN_MAX_FRAME 96  /* safe upper bound for app payloads */
#endif

/* ===== Discovery / health-check ===== */
#define WSAN_T_QUERY_MS 3000 /* default Query period (ms) */

/* ===== Gossip / Avalanche retransmission (broadcast)
 * Fan-out = how many times each node will rebroadcast a NEW frame
 * (with small jitter) to improve reachability. Dedup ensures one
 * fan-out cycle per (OriginID,MsgID).
 */
#define WSAN_T_GOSSIP_MS 2500 /* DATA broadcast period w/o RSU */
#define WSAN_FANOUT_DATA 1    /* #rebroadcasts per node for DATA */
#define WSAN_FANOUT_EF   3    /* #rebroadcasts per node for EF/Finish */
#define WSAN_JITTER_MS   30   /* 0..JITTER ms random delay */

/* ===== Unicast reliability ===== */
#ifndef WSAN_UNICAST_RETRIES
#define WSAN_UNICAST_RETRIES 3 /* total sends per DATA (1+retries) */
#endif

/* ===== Carry store sizing ===== */
#ifndef WSAN_CACHE_MAX_ITEMS
#define WSAN_CACHE_MAX_ITEMS 64 /* mobile's carry store count-limited */
#endif

/* ===== Direction (octants) for EF payload
 * Encoded as 1 byte 0..7: {N, NE, E, SE, S, SW, W, NW}.
 */
#define WSAN_DIR_E   0
#define WSAN_DIR_NE  1
#define WSAN_DIR_N   2
#define WSAN_DIR_NW  3
#define WSAN_DIR_W   4
#define WSAN_DIR_SW  5
#define WSAN_DIR_S   6
#define WSAN_DIR_SE  7
#define WSAN_DIR_UNK 255 /* unknown / not moving */

#endif /* WSAN_COMMON_H_ */