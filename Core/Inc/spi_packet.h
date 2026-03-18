#ifndef SPI_PACKET_H
#define SPI_PACKET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "n2k_raw_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_PACKET_SOF1 0xA5u
#define SPI_PACKET_SOF2 0x5Au
#define SPI_PACKET_TYPE_N2K_RX_FRAME 0x01u
#define SPI_PACKET_TYPE_N2K_TX_FRAME 0x02u
#define SPI_PACKET_TYPE_STATUS 0x03u
#define SPI_PACKET_PAYLOAD_LEN 18u
#define SPI_PACKET_TOTAL_LEN (2u + 1u + 1u + SPI_PACKET_PAYLOAD_LEN + 1u)

bool SPI_Packet_BuildFramePacket(uint8_t pkt_type, const N2K_RawFrame_t *frame, uint8_t *out_buf, size_t out_buf_size, size_t *out_len);
bool SPI_Packet_ParseFramePacket(const uint8_t *packet, size_t packet_len, uint8_t expected_type, N2K_RawFrame_t *out_frame);

#ifdef __cplusplus
}
#endif

#endif
