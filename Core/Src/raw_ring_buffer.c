#include "raw_ring_buffer.h"

static uint16_t raw_ring_next_index(uint16_t index, uint16_t capacity) {
    index++;
    if (index >= capacity) {
        index = 0;
    }
    return index;
}

void RawRingBuffer_Init(RawRingBuffer_t *rb, N2K_RawFrame_t *storage, uint16_t capacity) {
    rb->head = 0;
    rb->tail = 0;
    rb->capacity = capacity;
    rb->storage = storage;
}

bool RawRingBuffer_Push(RawRingBuffer_t *rb, const N2K_RawFrame_t *frame) {
    const uint16_t head = rb->head;
    const uint16_t next = raw_ring_next_index(head, rb->capacity);
    if (next == rb->tail) {
        return false;
    }

    rb->storage[head] = *frame;
    rb->head = next;
    return true;
}

bool RawRingBuffer_Pop(RawRingBuffer_t *rb, N2K_RawFrame_t *frame) {
    const uint16_t tail = rb->tail;
    if (tail == rb->head) {
        return false;
    }

    *frame = rb->storage[tail];
    rb->tail = raw_ring_next_index(tail, rb->capacity);
    return true;
}

size_t RawRingBuffer_Count(const RawRingBuffer_t *rb) {
    const uint16_t head = rb->head;
    const uint16_t tail = rb->tail;
    if (head >= tail) {
        return (size_t)(head - tail);
    }
    return (size_t)(rb->capacity - tail + head);
}
