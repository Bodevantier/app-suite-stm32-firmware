#include "spi_packet.h"

#include <string.h>
#include <stdint.h>

static uint8_t spi_packet_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00u;
    size_t i;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

static void write_u32_le(uint8_t *dst, uint32_t value) {
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8u) & 0xFFu);
    dst[2] = (uint8_t)((value >> 16u) & 0xFFu);
    dst[3] = (uint8_t)((value >> 24u) & 0xFFu);
}

static uint32_t read_u32_le(const uint8_t *src) {
    return ((uint32_t)src[0]) |
           ((uint32_t)src[1] << 8u) |
           ((uint32_t)src[2] << 16u) |
           ((uint32_t)src[3] << 24u);
}

bool SPI_Packet_BuildFramePacket(uint8_t pkt_type, const N2K_RawFrame_t *frame, uint8_t *out_buf, size_t out_buf_size, size_t *out_len) {
    size_t i = 0;
    uint8_t crc;

    if ((frame == 0) || (out_buf == 0) || (out_len == 0)) {
        return false;
    }
    if (out_buf_size < SPI_PACKET_TOTAL_LEN) {
        return false;
    }

    out_buf[i++] = SPI_PACKET_SOF1;
    out_buf[i++] = SPI_PACKET_SOF2;
    out_buf[i++] = pkt_type;
    out_buf[i++] = SPI_PACKET_PAYLOAD_LEN;

    write_u32_le(&out_buf[i], frame->timestamp_ms);
    i += 4;
    write_u32_le(&out_buf[i], frame->can_id);
    i += 4;
    out_buf[i++] = frame->dlc;
    out_buf[i++] = frame->flags;
    memcpy(&out_buf[i], frame->data, sizeof(frame->data));
    i += sizeof(frame->data);

    crc = spi_packet_crc8(&out_buf[2], 2u + SPI_PACKET_PAYLOAD_LEN);
    out_buf[i++] = crc;

    *out_len = i;
    return true;
}

bool SPI_Packet_ParseFramePacket(const uint8_t *packet, size_t packet_len, uint8_t expected_type, N2K_RawFrame_t *out_frame) {
    uint8_t crc;
    uint8_t dlc;

    if ((packet == 0) || (out_frame == 0)) {
        return false;
    }
    if (packet_len != SPI_PACKET_TOTAL_LEN) {
        return false;
    }
    if ((packet[0] != SPI_PACKET_SOF1) || (packet[1] != SPI_PACKET_SOF2)) {
        return false;
    }
    if (packet[2] != expected_type) {
        return false;
    }
    if (packet[3] != SPI_PACKET_PAYLOAD_LEN) {
        return false;
    }

    crc = spi_packet_crc8(&packet[2], 2u + SPI_PACKET_PAYLOAD_LEN);
    if (crc != packet[SPI_PACKET_TOTAL_LEN - 1u]) {
        return false;
    }

    out_frame->timestamp_ms = read_u32_le(&packet[4]);
    out_frame->can_id = read_u32_le(&packet[8]);
    dlc = packet[12];
    out_frame->dlc = dlc;
    out_frame->flags = packet[13];
    memcpy(out_frame->data, &packet[14], sizeof(out_frame->data));
    return true;
}

bool SPI_Packet_BuildCustomPacket(uint8_t pkt_type,
                                  const uint8_t *payload,
                                  uint8_t payload_len,
                                  uint8_t *out_buf,
                                  size_t out_buf_size,
                                  size_t *out_len) {
    size_t i = 0u;
    uint8_t crc;
    size_t total_len;

    if ((out_buf == 0) || (out_len == 0)) {
        return false;
    }
    if ((payload_len > 0u) && (payload == 0)) {
        return false;
    }
    if (payload_len > SPI_PACKET_MAX_PAYLOAD_LEN) {
        return false;
    }

    total_len = 2u + 1u + 1u + (size_t)payload_len + 1u;
    if (out_buf_size < total_len) {
        return false;
    }

    out_buf[i++] = SPI_PACKET_SOF1;
    out_buf[i++] = SPI_PACKET_SOF2;
    out_buf[i++] = pkt_type;
    out_buf[i++] = payload_len;

    if (payload_len > 0u) {
        memcpy(&out_buf[i], payload, payload_len);
        i += payload_len;
    }

    crc = spi_packet_crc8(&out_buf[2], (size_t)2u + (size_t)payload_len);
    out_buf[i++] = crc;
    *out_len = i;
    return true;
}

bool SPI_Packet_BuildBoatStatePacket(float aws_60s, float aws_5min, float aws_30min,
                                     float tws_60s, float tws_5min, float tws_30min,
                                     float vmg_ms,
                                     float live_aws_mps, float live_awa_deg,
                                     uint8_t *out_buf, size_t out_buf_size,
                                     size_t *out_len) {
    uint8_t payload[SPI_PACKET_BOAT_STATE_PAYLOAD_LEN];
    size_t i = 0u;
    float values[9];
    uint8_t vi;
    uint8_t crc;
    size_t total_len;

    if ((out_buf == 0) || (out_len == 0)) {
        return false;
    }

    total_len = 2u + 1u + 1u + SPI_PACKET_BOAT_STATE_PAYLOAD_LEN + 1u;
    if (out_buf_size < total_len) {
        return false;
    }

    values[0] = aws_60s;
    values[1] = aws_5min;
    values[2] = aws_30min;
    values[3] = tws_60s;
    values[4] = tws_5min;
    values[5] = tws_30min;
    values[6] = vmg_ms;
    values[7] = live_aws_mps;
    values[8] = live_awa_deg;

    /* Serialise each float as 4 bytes little-endian via memcpy to avoid
     * strict-aliasing issues with direct pointer casts. */
    for (vi = 0u; vi < 9u; vi++) {
        memcpy(&payload[vi * 4u], &values[vi], 4u);
    }

    out_buf[i++] = SPI_PACKET_SOF1;
    out_buf[i++] = SPI_PACKET_SOF2;
    out_buf[i++] = SPI_PACKET_TYPE_BOAT_STATE;
    out_buf[i++] = SPI_PACKET_BOAT_STATE_PAYLOAD_LEN;
    memcpy(&out_buf[i], payload, SPI_PACKET_BOAT_STATE_PAYLOAD_LEN);
    i += SPI_PACKET_BOAT_STATE_PAYLOAD_LEN;
    crc = spi_packet_crc8(&out_buf[2], 2u + SPI_PACKET_BOAT_STATE_PAYLOAD_LEN);
    out_buf[i++] = crc;
    *out_len = i;
    return true;
}
