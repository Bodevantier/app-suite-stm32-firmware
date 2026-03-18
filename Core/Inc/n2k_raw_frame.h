#ifndef N2K_RAW_FRAME_H
#define N2K_RAW_FRAME_H

#include <stdint.h>

typedef struct {
    uint32_t timestamp_ms;
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[8];
    uint8_t flags;
} N2K_RawFrame_t;

#endif
