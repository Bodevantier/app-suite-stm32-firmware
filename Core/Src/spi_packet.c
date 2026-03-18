#include "spi_packet.h"

#include <string.h>

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
