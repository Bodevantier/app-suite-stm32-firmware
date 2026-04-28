#ifndef N2K_RAW_BRIDGE_H
#define N2K_RAW_BRIDGE_H

#include <stdint.h>

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t can_rx_frames;
    uint32_t spi_tx_frames;
    uint32_t spi_rx_packets;
    uint32_t spi_queue_overflow;
    uint32_t can_tx_frames;
    uint32_t can_rx_overflow;
    uint32_t spi_parse_errors;
    uint32_t can_tx_errors;
} N2K_RawBridgeStats_t;

typedef struct {
    uint32_t can_id;
    uint32_t pgn;
    uint32_t timestamp_ms;
    uint8_t src;
    uint8_t dst;
    uint8_t priority;
    uint8_t dlc;
    uint8_t data[8];
} N2K_RawBridgeRxEvent_t;

typedef struct {
    uint32_t last_can_rx_pgn;
    uint8_t last_can_rx_src;
    uint8_t last_can_rx_dst;
    uint8_t last_can_rx_dlc;
    uint8_t last_can_rx_data[8];
    uint32_t can_rx_pgn_updates;
    uint32_t last_spi_to_can_pgn;
    uint8_t last_spi_to_can_src;
    uint8_t last_spi_to_can_dst;
    uint32_t spi_to_can_pgn_updates;
} N2K_RawBridgePgnDebug_t;

#define N2K_ASSEMBLED_DATA_MAX 223u
typedef struct {
    uint32_t pgn;
    uint16_t len;
    uint8_t  src;
    uint8_t  data[N2K_ASSEMBLED_DATA_MAX];
} N2K_RawBridgeAssembledEvent_t;

void N2K_RawBridge_Init(CAN_HandleTypeDef *hcan, SPI_HandleTypeDef *hspi);
void N2K_RawBridge_Process(void);
N2K_RawBridgeStats_t N2K_RawBridge_GetStats(void);
N2K_RawBridgePgnDebug_t N2K_RawBridge_GetPgnDebug(void);
uint8_t N2K_RawBridge_PopLogEvent(N2K_RawBridgeRxEvent_t *event_out);
uint8_t N2K_RawBridge_SendIsoRequest(uint8_t src, uint8_t dst, uint32_t requested_pgn);
uint8_t N2K_RawBridge_QueueSpiPacket(uint8_t pkt_type, const uint8_t *payload, uint8_t payload_len);
uint8_t N2K_RawBridge_PopAssembledEvent(N2K_RawBridgeAssembledEvent_t *event_out);
void    N2K_RawBridge_ResetSeenSources(void);
uint8_t N2K_RawBridge_PopNewSource(uint8_t *src_out);

#ifdef __cplusplus
}
#endif

#endif
