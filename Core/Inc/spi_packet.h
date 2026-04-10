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
#define SPI_PACKET_TYPE_BOAT_STATE 0x04u
#define SPI_PACKET_TYPE_DEVICE_LIST 0x10u
#define SPI_PACKET_TYPE_DEVICE_LIST_REQUEST 0x11u
#define SPI_PACKET_PAYLOAD_LEN 18u
#define SPI_PACKET_TOTAL_LEN (2u + 1u + 1u + SPI_PACKET_PAYLOAD_LEN + 1u)
#define SPI_PACKET_MAX_PAYLOAD_LEN 250u

/* BOAT_STATE payload layout (36 bytes, all float32 little-endian, NaN = not ready):
 *   [0..3]   AWS avg 60 s          (m/s)
 *   [4..7]   AWS avg 5 min         (m/s)
 *   [8..11]  AWS avg 30 min        (m/s)
 *   [12..15] TWS avg 60 s          (m/s)
 *   [16..19] TWS avg 5 min         (m/s)
 *   [20..23] TWS avg 30 min        (m/s)
 *   [24..27] VMG to wind           (m/s, positive = upwind, NaN = invalid)
 *   [28..31] Live Apparent Wind Speed (m/s, NaN = insufficient data)
 *   [32..35] Live Apparent Wind Angle (degrees 0-360 from bow, NaN = insufficient data)
 */
#define SPI_PACKET_BOAT_STATE_PAYLOAD_LEN 36u

bool SPI_Packet_BuildFramePacket(uint8_t pkt_type, const N2K_RawFrame_t *frame, uint8_t *out_buf, size_t out_buf_size, size_t *out_len);
bool SPI_Packet_ParseFramePacket(const uint8_t *packet, size_t packet_len, uint8_t expected_type, N2K_RawFrame_t *out_frame);
bool SPI_Packet_BuildCustomPacket(uint8_t pkt_type, const uint8_t *payload, uint8_t payload_len, uint8_t *out_buf, size_t out_buf_size, size_t *out_len);
bool SPI_Packet_BuildBoatStatePacket(float aws_60s, float aws_5min, float aws_30min,
                                     float tws_60s, float tws_5min, float tws_30min,
                                     float vmg_ms,
                                     float live_aws_mps, float live_awa_deg,
                                     uint8_t *out_buf, size_t out_buf_size, size_t *out_len);

#ifdef __cplusplus
}
#endif

#endif
