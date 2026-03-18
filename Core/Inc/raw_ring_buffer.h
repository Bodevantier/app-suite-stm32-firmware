#ifndef RAW_RING_BUFFER_H
#define RAW_RING_BUFFER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "n2k_raw_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    uint16_t capacity;
    N2K_RawFrame_t *storage;
} RawRingBuffer_t;

void RawRingBuffer_Init(RawRingBuffer_t *rb, N2K_RawFrame_t *storage, uint16_t capacity);
bool RawRingBuffer_Push(RawRingBuffer_t *rb, const N2K_RawFrame_t *frame);
bool RawRingBuffer_Pop(RawRingBuffer_t *rb, N2K_RawFrame_t *frame);
size_t RawRingBuffer_Count(const RawRingBuffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif
