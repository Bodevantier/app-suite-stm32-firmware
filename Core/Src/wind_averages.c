#include "wind_averages.h"

#include <math.h>
#include <string.h>

/* -----------------------------------------------------------------------
 * Helpers
 * ----------------------------------------------------------------------- */

float WindAverages_NaN(void) {
    union { uint32_t u; float f; } v;
    v.u = 0x7FC00000u;
    return v.f;
}

bool WindAverages_IsNaN(float f) {
    return (f != f);
}

static void ring_init(WindRing_t *r, uint8_t capacity,
                      uint32_t subsample_interval_ms) {
    memset(r, 0, sizeof(*r));
    r->capacity              = capacity;
    r->subsample_interval_ms = subsample_interval_ms;
    /* Mark all entries invalid */
    for (uint8_t i = 0u; i < capacity; i++) {
        r->entries[i].speed_u16 = 0xFFFFu;
    }
}

/* Push a speed sample (m/s) into a single ring, honouring the subsample
 * interval.  Returns 1 if the sample was accepted, 0 if skipped. */
static uint8_t ring_push(WindRing_t *r, float speed_mps, uint32_t now_ms) {
    uint32_t age_since_last;

    if (r->count > 0u) {
        age_since_last = now_ms - r->last_add_ms;
        if (age_since_last < r->subsample_interval_ms) {
            return 0u; /* too soon */
        }
    }

    /* Encode speed as uint16 (speed × 100, saturate at 0xFFFE) */
    uint32_t enc = (uint32_t)(speed_mps * 100.0f + 0.5f);
    if (enc > 0xFFFEu) { enc = 0xFFFEu; }

    /* Age in ticks (seconds since base_epoch). We store ticks modulo 65536
     * to fit uint16.  Only used for rough staleness detection so wrap is ok. */
    uint16_t age_tick = (uint16_t)((now_ms - r->base_epoch_ms) / 1000u);

    r->entries[r->head].speed_u16 = (uint16_t)enc;
    r->entries[r->head].age_ticks = age_tick;
    r->head = (uint8_t)((r->head + 1u) % r->capacity);
    if (r->count < r->capacity) {
        r->count++;
    }
    r->last_add_ms = now_ms;
    return 1u;
}

/* Compute the mean of all samples in the ring whose age_tick falls within
 * [current_tick - window_ticks, current_tick].
 * Returns NaN if fewer than 2 valid in-window samples exist. */
static float ring_window_avg(const WindRing_t *r, uint32_t now_ms,
                             uint32_t window_ms) {
    uint16_t current_tick = (uint16_t)((now_ms - r->base_epoch_ms) / 1000u);
    uint16_t window_ticks = (uint16_t)(window_ms / 1000u);
    float    sum          = 0.0f;
    uint8_t  n            = 0u;

    for (uint8_t i = 0u; i < r->count; i++) {
        /* Walk from newest to oldest */
        uint8_t k = (uint8_t)((r->head + r->capacity - 1u - i) % r->capacity);
        uint16_t s = r->entries[k].speed_u16;
        if (s == 0xFFFFu) { continue; } /* invalid marker */

        /* Age check (uint16 subtraction handles wrap correctly for ≤18h) */
        uint16_t age = (uint16_t)(current_tick - r->entries[k].age_ticks);
        if (age > window_ticks) {
            break; /* older entries are further out of window */
        }
        sum += (float)s / 100.0f;
        n++;
    }
    return (n >= 2u) ? (sum / (float)n) : WindAverages_NaN();
}

static void wind_type_avg_init(WindTypeAvg_t *wt) {
    ring_init(&wt->fast, WIND_AVG_FAST_CAPACITY, 1000u);   /* 1 s */
    ring_init(&wt->mid,  WIND_AVG_MID_CAPACITY,  5000u);   /* 5 s */
    ring_init(&wt->slow, WIND_AVG_SLOW_CAPACITY, 30000u);  /* 30 s */
    wt->avg_60s   = WindAverages_NaN();
    wt->avg_5min  = WindAverages_NaN();
    wt->avg_30min = WindAverages_NaN();
}

static void wind_type_add(WindTypeAvg_t *wt, float speed_mps, uint32_t now_ms) {
    ring_push(&wt->fast, speed_mps, now_ms);
    ring_push(&wt->mid,  speed_mps, now_ms);
    ring_push(&wt->slow, speed_mps, now_ms);
}

static void wind_type_recompute(WindTypeAvg_t *wt, uint32_t now_ms) {
    wt->avg_60s   = ring_window_avg(&wt->fast, now_ms, 60000u);
    wt->avg_5min  = ring_window_avg(&wt->mid,  now_ms, 300000u);
    wt->avg_30min = ring_window_avg(&wt->slow, now_ms, 1800000u);
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void WindAverages_Init(WindAverages_t *wa) {
    if (wa == 0) { return; }
    memset(wa, 0, sizeof(*wa));
    wind_type_avg_init(&wa->aws);
    wind_type_avg_init(&wa->tws);
    wa->vmg_ms   = WindAverages_NaN();
    wa->sog_valid = false;
}

void WindAverages_AddAws(WindAverages_t *wa, float aws_mps, uint32_t now_ms) {
    if (wa == 0) { return; }
    wind_type_add(&wa->aws, aws_mps, now_ms);
}

void WindAverages_AddTws(WindAverages_t *wa, float tws_mps, uint32_t now_ms) {
    if (wa == 0) { return; }
    wind_type_add(&wa->tws, tws_mps, now_ms);
}

void WindAverages_AddSog(WindAverages_t *wa, float sog_mps, uint32_t now_ms) {
    if (wa == 0) { return; }
    wa->last_sog_ms    = sog_mps;
    wa->sog_valid      = true;
    wa->last_sog_ms_ts = now_ms;
}

void WindAverages_AddTwa(WindAverages_t *wa, float twa_deg) {
    float twa_rad;
    float vmg;

    if (wa == 0) { return; }

    if (!wa->sog_valid || wa->last_sog_ms < WIND_VMG_MIN_SOG_MS) {
        wa->vmg_ms = WindAverages_NaN();
        return;
    }

    twa_rad    = twa_deg * 3.14159265f / 180.0f;
    vmg        = wa->last_sog_ms * cosf(twa_rad);
    wa->vmg_ms = vmg;
}

void WindAverages_Recompute(WindAverages_t *wa, uint32_t now_ms) {
    if (wa == 0) { return; }
    wind_type_recompute(&wa->aws, now_ms);
    wind_type_recompute(&wa->tws, now_ms);
}

