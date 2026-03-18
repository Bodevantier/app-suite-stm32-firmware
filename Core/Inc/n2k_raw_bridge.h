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
    uint32_t can_tx_frames;
    uint32_t can_rx_overflow;
    uint32_t spi_parse_errors;
    uint32_t can_tx_errors;
} N2K_RawBridgeStats_t;

void N2K_RawBridge_Init(CAN_HandleTypeDef *hcan, SPI_HandleTypeDef *hspi);
void N2K_RawBridge_Process(void);
N2K_RawBridgeStats_t N2K_RawBridge_GetStats(void);

#ifdef __cplusplus
}
#endif

#endif
