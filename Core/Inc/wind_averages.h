#ifndef WIND_AVERAGES_H
#define WIND_AVERAGES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * wind_averages — Rolling time-window averages for AWS / TWS and VMG.
 *
 * All speed values are stored and returned in m/s.
 * A NaN sentinel (IEEE-754 quiet NaN) signals "not yet ready" so callers
 * can distinguish a real zero from missing data.
 *
 * RAM budget (STM32F103xB = 20 KB SRAM):
 *
 *   Each value is stored as a uint16_t (speed × 100, capped at 655 m/s
 *   which is well beyond any real wind) together with a uint16_t
 *   sub-second counter so the age can be derived from a shared base timer.
 *   This halves the per-sample cost compared to float + uint32.
 *
 *   Three tier rings per wind type:
 *     fast  – 80 entries, one sample every 1 s  → covers 80 s  (≥60 s avg)
 *     mid   – 60 entries, subsample every 5 s   → covers 300 s (≥5 min avg)
 *     slow  – 60 entries, subsample every 30 s  → covers 1800 s (≥30 min avg)
 *
 *   Two wind types (AWS, TWS) × 3 rings × 62 entries × 4 bytes = ~1.5 KB.
 * ----------------------------------------------------------------------- */

#define WIND_AVG_FAST_CAPACITY  62u   /* 62 s at 1 Hz — covers 60 s avg   */
#define WIND_AVG_MID_CAPACITY   62u   /* 62 slots × 5 s = 310 s coverage   */
#define WIND_AVG_SLOW_CAPACITY  62u   /* 62 slots × 30 s = 1860 s coverage */

/* Minimum valid SOG to include in VMG calculation (m/s). */
#define WIND_VMG_MIN_SOG_MS 0.1f

/* ---- Internal sample type (4 bytes) ---------------------------------- */
typedef struct {
    uint16_t speed_u16;     /* speed × 100 (0.01 m/s per LSB), 0xFFFF = invalid */
    uint16_t age_ticks;     /* ticks (ms / 1000) since ring base epoch — wraps  */
} WindSample_t;

/* ---- Single ring buffer ---------------------------------------------- */
/* All three tiers use the same capacity (62) — this keeps the struct
 * uniform while still covering every required window:
 *   fast: 62 s  ≥ 60 s window
 *   mid : 62×5 s = 310 s ≥ 300 s (5 min) window
 *   slow: 62×30 s = 1860 s ≥ 1800 s (30 min) window            */
#define WIND_AVG_RING_CAPACITY  WIND_AVG_FAST_CAPACITY

typedef struct {
    WindSample_t  entries[WIND_AVG_RING_CAPACITY];
    uint8_t       head;                          /* next write index          */
    uint8_t       count;                         /* valid entries (≤ capacity) */
    uint8_t       capacity;                      /* set by ring_init()         */
    uint32_t      base_epoch_ms;
    uint32_t      last_add_ms;
    uint32_t      subsample_interval_ms;
} WindRing_t;

/* ---- Per-wind-type structure ----------------------------------------- */
typedef struct {
    WindRing_t fast;
    WindRing_t mid;
    WindRing_t slow;
    float      avg_60s;    /* cached averages; NaN = not ready                */
    float      avg_5min;
    float      avg_30min;
} WindTypeAvg_t;

/* ---- Top-level state -------------------------------------------------- */
typedef struct {
    WindTypeAvg_t aws;
    WindTypeAvg_t tws;
    float         vmg_ms;        /* NaN = invalid / insufficient data         */
    float         last_sog_ms;
    bool          sog_valid;
    uint32_t      last_sog_ms_ts;
} WindAverages_t;

/* ---- Public API -------------------------------------------------------- */

void WindAverages_Init(WindAverages_t *wa);

/* Feed a new apparent-wind speed sample (m/s) at the given HAL tick. */
void WindAverages_AddAws(WindAverages_t *wa, float aws_mps, uint32_t now_ms);

/* Feed a new true-wind speed sample (m/s) at the given HAL tick. */
void WindAverages_AddTws(WindAverages_t *wa, float tws_mps, uint32_t now_ms);

/* Feed a new SOG sample (m/s).  Stored for VMG computation. */
void WindAverages_AddSog(WindAverages_t *wa, float sog_mps, uint32_t now_ms);

/* Feed a new TRUE wind angle (degrees) to trigger VMG recomputation.
 * Call after AddTws so the most recent TWS and SOG values are used. */
void WindAverages_AddTwa(WindAverages_t *wa, float twa_deg);

/* Recompute all cached averages.  Call periodically (e.g. after each wind
 * sample).  Pass the current HAL_GetTick(). */
void WindAverages_Recompute(WindAverages_t *wa, uint32_t now_ms);

/* Return the IEEE-754 quiet NaN sentinel. */
float WindAverages_NaN(void);

/* Return true if f is NaN. */
bool WindAverages_IsNaN(float f);

#ifdef __cplusplus
}
#endif

#endif /* WIND_AVERAGES_H */

