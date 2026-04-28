#include "n2k_raw_bridge.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "raw_ring_buffer.h"
#include "spi_packet.h"
#include "usart.h"
#include "devicelist_handler.h"

#define RAW_BRIDGE_RING_CAPACITY 128u
#define RAW_FRAME_FLAG_EXT_ID 0x01u
#define RAW_FRAME_FLAG_RTR 0x02u
#define RAW_FRAME_FLAG_DIRECTION 0x04u
#define RAW_BRIDGE_SPI_POLL_BYTES SPI_PACKET_TOTAL_LEN
#define RAW_BRIDGE_SPI_MAX_PACKET_LEN 260u
#define RAW_BRIDGE_DEBUG_UART 0u
#define RAW_BRIDGE_DEBUG_LINE_LEN 320u
#define RAW_BRIDGE_FAST_PACKET_SLOTS 3u
#define RAW_BRIDGE_FAST_PACKET_MAX_DATA_LEN 223u
#define N2K_PGN_ISO_REQUEST 59904ul
#define N2K_PGN_ISO_ADDRESS_CLAIM 60928ul
#define N2K_PGN_PRODUCT_INFORMATION 126996ul
#define N2K_PGN_CONFIGURATION_INFORMATION 126998ul
#define N2K_PGN_PGN_LIST 126464ul

static CAN_HandleTypeDef *s_hcan = 0;
static SPI_HandleTypeDef *s_hspi = 0;
static RawRingBuffer_t s_ring;
static N2K_RawFrame_t s_ring_storage[RAW_BRIDGE_RING_CAPACITY];
static volatile N2K_RawBridgeStats_t s_stats = {0};
static volatile N2K_RawBridgePgnDebug_t s_pgn_debug = {
    .last_can_rx_pgn = UINT32_MAX,
    .last_spi_to_can_pgn = UINT32_MAX
};

typedef enum {
    SPI_PARSE_WAIT_SOF1 = 0,
    SPI_PARSE_WAIT_SOF2,
    SPI_PARSE_READ_TYPE,
    SPI_PARSE_READ_LEN,
    SPI_PARSE_READ_PAYLOAD_CRC
} SpiParseState_t;

typedef struct {
    SpiParseState_t state;
    uint8_t packet[RAW_BRIDGE_SPI_MAX_PACKET_LEN];
    uint16_t index;
    uint16_t expected_len;
} SpiParser_t;

typedef struct {
    uint8_t in_use;
    uint8_t is_ble_to_n2k;
    uint8_t source;
    uint8_t sequence_id;
    uint8_t next_frame_index;
    uint32_t pgn;
    uint16_t expected_len;
    uint16_t data_len;
    uint8_t data[RAW_BRIDGE_FAST_PACKET_MAX_DATA_LEN];
} RawBridgeFastPacketSlot_t;

static SpiParser_t s_spi_parser = {0};
#if RAW_BRIDGE_DEBUG_UART
static RawBridgeFastPacketSlot_t s_fast_packet_slots[RAW_BRIDGE_FAST_PACKET_SLOTS] __attribute__((unused)) = {0};
#endif

typedef struct {
    uint16_t len;
    uint8_t bytes[RAW_BRIDGE_SPI_MAX_PACKET_LEN];
} RawBridgeSpiQueuedPacket_t;

#define RAW_BRIDGE_SPI_QUEUE_CAPACITY 24u
static RawBridgeSpiQueuedPacket_t s_spi_queue[RAW_BRIDGE_SPI_QUEUE_CAPACITY];
static volatile uint8_t s_spi_queue_head = 0u;
static volatile uint8_t s_spi_queue_tail = 0u;

/* Separate log-only event queue: all extended-ID PGNs, used only for UART
 * debug printing in the main loop. Kept small — main loop drains it quickly
 * and we rate-limit high-frequency PGNs in the consumer. */
#define RAW_BRIDGE_LOG_EVENT_CAPACITY 16u
static N2K_RawBridgeRxEvent_t s_log_events[RAW_BRIDGE_LOG_EVENT_CAPACITY];
static volatile uint16_t s_log_event_head = 0u;
static volatile uint16_t s_log_event_tail = 0u;

/* ── Always-on identity fast-packet assembler ──────────────────────────── */
#define RAW_BRIDGE_ID_FP_SLOTS 3u
static RawBridgeFastPacketSlot_t s_id_fp_slots[RAW_BRIDGE_ID_FP_SLOTS] = {0};

/* ── Assembled event FIFO (N2K identity fast-packets, main-loop only) ──── */
#define RAW_BRIDGE_ASSEMBLED_QUEUE_LEN 3u
static N2K_RawBridgeAssembledEvent_t s_assembled_events[RAW_BRIDGE_ASSEMBLED_QUEUE_LEN];
static uint8_t s_assembled_head = 0u;
static uint8_t s_assembled_tail = 0u;

/* ── Seen-sources: bitmask + new-source FIFO ───────────────────────────── */
#define RAW_BRIDGE_SOURCE_MASK_BYTES 32u   /* 256 sources / 8 bits each */
static uint8_t s_seen_sources[RAW_BRIDGE_SOURCE_MASK_BYTES] = {0};
#define RAW_BRIDGE_NEW_SOURCE_QUEUE_LEN 32u
static uint8_t s_new_source_queue[RAW_BRIDGE_NEW_SOURCE_QUEUE_LEN] = {0};
static volatile uint8_t s_new_source_head = 0u;
static volatile uint8_t s_new_source_tail = 0u;

#if RAW_BRIDGE_DEBUG_UART
static void raw_bridge_uart_print(const char *text) {
#if RAW_BRIDGE_DEBUG_UART
    HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 100u);
#else
    (void)text;
#endif
}

static uint16_t raw_bridge_read_u16_le(const uint8_t *src) {
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8u);
}

static uint32_t raw_bridge_read_u24_le(const uint8_t *src) {
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8u) |
           ((uint32_t)src[2] << 16u);
}

static uint64_t raw_bridge_read_u64_le(const uint8_t *src) {
    uint64_t value = 0u;
    uint8_t i;

    for (i = 0u; i < 8u; i++) {
        value |= ((uint64_t)src[i]) << (8u * i);
    }

    return value;
}

static void raw_bridge_copy_n2k_string(
    char *dst,
    size_t dst_size,
    const uint8_t *src,
    size_t src_len,
    uint8_t pad_char
) {
    size_t i;

    if ((dst == 0) || (dst_size == 0u)) {
        return;
    }

    for (i = 0u; i + 1u < dst_size && i < src_len; i++) {
        uint8_t value = src[i];
        if ((value == 0x00u) || (value == pad_char)) {
            break;
        }
        dst[i] = (char)value;
    }

    dst[i] = '\0';
}

static uint8_t raw_bridge_parse_var_string(
    const uint8_t *src,
    uint16_t total_len,
    uint16_t *index,
    char *dst,
    size_t dst_size
) {
    uint16_t offset;
    uint8_t field_len;
    uint8_t field_type;
    uint8_t text_len;

    if ((src == 0) || (index == 0)) {
        return 0u;
    }

    offset = *index;
    if ((uint16_t)(offset + 2u) > total_len) {
        return 0u;
    }

    field_len = src[offset++];
    field_type = src[offset++];
    if ((field_len < 2u) || (field_type != 0x01u)) {
        return 0u;
    }

    text_len = (uint8_t)(field_len - 2u);
    if ((uint16_t)(offset + text_len) > total_len) {
        return 0u;
    }

    raw_bridge_copy_n2k_string(dst, dst_size, &src[offset], text_len, 0xFFu);
    *index = (uint16_t)(offset + text_len);
    return 1u;
}

static uint32_t raw_bridge_get_pgn(const N2K_RawFrame_t *frame) {
    uint32_t can_id;
    uint32_t data_page;
    uint32_t pdu_format;
    uint32_t pdu_specific;

    if ((frame->flags & RAW_FRAME_FLAG_EXT_ID) == 0u) {
        return UINT32_MAX;
    }

    can_id = frame->can_id & 0x1FFFFFFFu;
    data_page = (can_id >> 24u) & 0x01u;
    pdu_format = (can_id >> 16u) & 0xFFu;
    pdu_specific = (can_id >> 8u) & 0xFFu;

    if (pdu_format < 240u) {
        pdu_specific = 0u;
    }

    return (data_page << 16u) | (pdu_format << 8u) | pdu_specific;
}

static uint8_t raw_bridge_get_source(const N2K_RawFrame_t *frame) {
    return (uint8_t)(frame->can_id & 0xFFu);
}

static uint8_t raw_bridge_get_destination(const N2K_RawFrame_t *frame) {
    uint32_t pdu_format;

    if ((frame->flags & RAW_FRAME_FLAG_EXT_ID) == 0u) {
        return 0xFFu;
    }

    pdu_format = (frame->can_id >> 16u) & 0xFFu;
    if (pdu_format < 240u) {
        return (uint8_t)((frame->can_id >> 8u) & 0xFFu);
    }

    return 0xFFu;
}

static uint32_t raw_bridge_get_requested_pgn(const N2K_RawFrame_t *frame) {
    if (frame->dlc < 3u) {
        return UINT32_MAX;
    }

    return (uint32_t)frame->data[0] |
           ((uint32_t)frame->data[1] << 8u) |
           ((uint32_t)frame->data[2] << 16u);
}

static uint8_t raw_bridge_is_device_list_pgn(uint32_t pgn) {
    switch (pgn) {
        case N2K_PGN_ISO_ADDRESS_CLAIM:
        case N2K_PGN_PRODUCT_INFORMATION:
        case N2K_PGN_CONFIGURATION_INFORMATION:
        case N2K_PGN_PGN_LIST:
            return 1u;

        default:
            return 0u;
    }
}

static const char *raw_bridge_device_list_pgn_name(uint32_t pgn) {
    switch (pgn) {
        case N2K_PGN_ISO_REQUEST:
            return "ISORequest";

        case N2K_PGN_ISO_ADDRESS_CLAIM:
            return "AddressClaim";

        case N2K_PGN_PRODUCT_INFORMATION:
            return "ProductInfo";

        case N2K_PGN_CONFIGURATION_INFORMATION:
            return "ConfigInfo";

        case N2K_PGN_PGN_LIST:
            return "PGNList";

        default:
            return "Other";
    }
}

static const char *raw_bridge_pgn_list_name(uint8_t list_type) {
    switch (list_type) {
        case 0u:
            return "Transmit";

        case 1u:
            return "Receive";

        default:
            return "Unknown";
    }
}

static void raw_bridge_uart_print_line(const char *line) {
#if RAW_BRIDGE_DEBUG_UART
    raw_bridge_uart_print(line);
#else
    (void)line;
#endif
}

static void raw_bridge_log_iso_request(
    const char *tag,
    const N2K_RawFrame_t *frame,
    uint32_t requested_pgn
) {
#if RAW_BRIDGE_DEBUG_UART
    char line[RAW_BRIDGE_DEBUG_LINE_LEN];

    (void)snprintf(
        line,
        sizeof(line),
        "%s ISORequest req_pgn=59904 src=%u dst=%u requested_pgn=%lu(0x%06lX) requested_name=%s\r\n",
        tag,
        (unsigned int)raw_bridge_get_source(frame),
        (unsigned int)raw_bridge_get_destination(frame),
        (unsigned long)requested_pgn,
        (unsigned long)requested_pgn,
        raw_bridge_device_list_pgn_name(requested_pgn)
    );
    raw_bridge_uart_print_line(line);
#else
    (void)tag;
    (void)frame;
    (void)requested_pgn;
#endif
}

static void raw_bridge_log_address_claim(const char *tag, const N2K_RawFrame_t *frame) {
#if RAW_BRIDGE_DEBUG_UART
    char line[RAW_BRIDGE_DEBUG_LINE_LEN];
    uint64_t name;
    uint32_t unique_number;
    uint16_t manufacturer_code;
    uint8_t device_instance;
    uint8_t device_function;
    uint8_t device_class;
    uint8_t system_instance;
    uint8_t industry_group;

    if (frame->dlc < 8u) {
        return;
    }

    name = raw_bridge_read_u64_le(frame->data);
    unique_number = (uint32_t)(name & 0x1FFFFFull);
    manufacturer_code = (uint16_t)((name >> 21u) & 0x7FFu);
    device_instance = (uint8_t)((name >> 32u) & 0xFFu);
    device_function = (uint8_t)((name >> 40u) & 0xFFu);
    device_class = (uint8_t)(((name >> 48u) & 0xFFu) >> 1u);
    system_instance = (uint8_t)((name >> 56u) & 0x0Fu);
    industry_group = (uint8_t)((name >> 60u) & 0x07u);

    (void)snprintf(
        line,
        sizeof(line),
        "%s AddressClaim src=%u name=0x%08lX%08lX unique=%lu manufacturer_code=%u device_instance=%u function=%u class=%u industry=%u system_instance=%u\r\n",
        tag,
        (unsigned int)raw_bridge_get_source(frame),
        (unsigned long)(name >> 32u),
        (unsigned long)(name & 0xFFFFFFFFul),
        (unsigned long)unique_number,
        (unsigned int)manufacturer_code,
        (unsigned int)device_instance,
        (unsigned int)device_function,
        (unsigned int)device_class,
        (unsigned int)industry_group,
        (unsigned int)system_instance
    );
    raw_bridge_uart_print_line(line);
#else
    (void)tag;
    (void)frame;
#endif
}

static void raw_bridge_log_product_information(
    const char *tag,
    uint8_t source,
    const uint8_t *payload,
    uint16_t payload_len
) {
#if RAW_BRIDGE_DEBUG_UART
    char line[RAW_BRIDGE_DEBUG_LINE_LEN];
    uint16_t n2k_version;
    uint16_t product_code;
    uint8_t certification_level;
    uint8_t load_equivalency;
    char model_id[33];
    char sw_code[33];
    char model_version[33];
    char serial_code[33];

    if (payload_len < 134u) {
        return;
    }

    n2k_version = raw_bridge_read_u16_le(&payload[0]);
    product_code = raw_bridge_read_u16_le(&payload[2]);
    raw_bridge_copy_n2k_string(model_id, sizeof(model_id), &payload[4], 32u, 0xFFu);
    raw_bridge_copy_n2k_string(sw_code, sizeof(sw_code), &payload[36], 32u, 0xFFu);
    raw_bridge_copy_n2k_string(model_version, sizeof(model_version), &payload[68], 32u, 0xFFu);
    raw_bridge_copy_n2k_string(serial_code, sizeof(serial_code), &payload[100], 32u, 0xFFu);
    certification_level = payload[132];
    load_equivalency = payload[133];

    (void)snprintf(
        line,
        sizeof(line),
        "%s ProductInfo src=%u model=\"%s\" sw=\"%s\" version=\"%s\" serial=\"%s\" product_code=%u n2k_version=%u cert=%u load_eq=%u\r\n",
        tag,
        (unsigned int)source,
        model_id,
        sw_code,
        model_version,
        serial_code,
        (unsigned int)product_code,
        (unsigned int)n2k_version,
        (unsigned int)certification_level,
        (unsigned int)load_equivalency
    );
    raw_bridge_uart_print_line(line);
#else
    (void)tag;
    (void)source;
    (void)payload;
    (void)payload_len;
#endif
}

static void raw_bridge_log_configuration_information(
    const char *tag,
    uint8_t source,
    const uint8_t *payload,
    uint16_t payload_len
) {
#if RAW_BRIDGE_DEBUG_UART
    char line[RAW_BRIDGE_DEBUG_LINE_LEN];
    uint16_t index = 0u;
    char installation_1[72];
    char installation_2[72];
    char manufacturer_info[72];

    if (!raw_bridge_parse_var_string(payload, payload_len, &index, installation_1, sizeof(installation_1))) {
        return;
    }
    if (!raw_bridge_parse_var_string(payload, payload_len, &index, installation_2, sizeof(installation_2))) {
        return;
    }
    if (!raw_bridge_parse_var_string(payload, payload_len, &index, manufacturer_info, sizeof(manufacturer_info))) {
        return;
    }

    (void)snprintf(
        line,
        sizeof(line),
        "%s ConfigInfo src=%u manufacturer=\"%s\" install1=\"%s\" install2=\"%s\"\r\n",
        tag,
        (unsigned int)source,
        manufacturer_info,
        installation_1,
        installation_2
    );
    raw_bridge_uart_print_line(line);
#else
    (void)tag;
    (void)source;
    (void)payload;
    (void)payload_len;
#endif
}

static void raw_bridge_log_pgn_list(
    const char *tag,
    uint8_t source,
    const uint8_t *payload,
    uint16_t payload_len
) {
#if RAW_BRIDGE_DEBUG_UART
    char line[RAW_BRIDGE_DEBUG_LINE_LEN];
    uint8_t list_type;
    uint16_t pgn_count;
    uint32_t first_pgn = UINT32_MAX;

    if (payload_len < 1u) {
        return;
    }

    list_type = payload[0];
    pgn_count = (uint16_t)((payload_len - 1u) / 3u);
    if (pgn_count > 0u) {
        first_pgn = raw_bridge_read_u24_le(&payload[1]);
    }

    if (first_pgn == UINT32_MAX) {
        (void)snprintf(
            line,
            sizeof(line),
            "%s PGNList src=%u list=%s count=%u\r\n",
            tag,
            (unsigned int)source,
            raw_bridge_pgn_list_name(list_type),
            (unsigned int)pgn_count
        );
    } else {
        (void)snprintf(
            line,
            sizeof(line),
            "%s PGNList src=%u list=%s count=%u first_pgn=%lu\r\n",
            tag,
            (unsigned int)source,
            raw_bridge_pgn_list_name(list_type),
            (unsigned int)pgn_count,
            (unsigned long)first_pgn
        );
    }
    raw_bridge_uart_print_line(line);
#else
    (void)tag;
    (void)source;
    (void)payload;
    (void)payload_len;
#endif
}

static RawBridgeFastPacketSlot_t *raw_bridge_fast_packet_find_slot(
    uint8_t is_ble_to_n2k,
    uint8_t source,
    uint32_t pgn,
    uint8_t sequence_id
) {
    uint8_t i;

    for (i = 0u; i < RAW_BRIDGE_FAST_PACKET_SLOTS; i++) {
        RawBridgeFastPacketSlot_t *slot = &s_fast_packet_slots[i];
        if ((slot->in_use != 0u) &&
            (slot->is_ble_to_n2k == is_ble_to_n2k) &&
            (slot->source == source) &&
            (slot->pgn == pgn) &&
            (slot->sequence_id == sequence_id)) {
            return slot;
        }
    }

    return 0;
}

static RawBridgeFastPacketSlot_t *raw_bridge_fast_packet_alloc_slot(void) {
    uint8_t i;

    for (i = 0u; i < RAW_BRIDGE_FAST_PACKET_SLOTS; i++) {
        if (s_fast_packet_slots[i].in_use == 0u) {
            return &s_fast_packet_slots[i];
        }
    }

    return &s_fast_packet_slots[0];
}

static void raw_bridge_fast_packet_reset_slot(RawBridgeFastPacketSlot_t *slot) {
    if (slot != 0) {
        memset(slot, 0, sizeof(*slot));
    }
}

static void raw_bridge_fast_packet_handle_complete(
    const char *tag,
    uint32_t pgn,
    uint8_t source,
    const uint8_t *payload,
    uint16_t payload_len
) {
    switch (pgn) {
        case N2K_PGN_PRODUCT_INFORMATION:
            raw_bridge_log_product_information(tag, source, payload, payload_len);
            break;

        case N2K_PGN_CONFIGURATION_INFORMATION:
            raw_bridge_log_configuration_information(tag, source, payload, payload_len);
            break;

        case N2K_PGN_PGN_LIST:
            raw_bridge_log_pgn_list(tag, source, payload, payload_len);
            break;

        default:
            break;
    }
}

static void raw_bridge_debug_fast_packet(const char *tag, const N2K_RawFrame_t *frame, uint8_t is_ble_to_n2k) {
    RawBridgeFastPacketSlot_t *slot;
    uint8_t frame_index;
    uint8_t sequence_id;
    uint8_t source;
    uint16_t copy_len;
    uint16_t payload_offset;
    uint16_t remaining;
    uint32_t pgn;

    if (frame->dlc == 0u) {
        return;
    }

    pgn = raw_bridge_get_pgn(frame);
    source = raw_bridge_get_source(frame);
    sequence_id = frame->data[0] >> 5u;
    frame_index = frame->data[0] & 0x1Fu;

    if (frame_index == 0u) {
        uint16_t expected_len;

        if (frame->dlc < 2u) {
            return;
        }

        expected_len = frame->data[1];
        if ((expected_len == 0u) || (expected_len > RAW_BRIDGE_FAST_PACKET_MAX_DATA_LEN)) {
            return;
        }

        slot = raw_bridge_fast_packet_find_slot(is_ble_to_n2k, source, pgn, sequence_id);
        if (slot == 0) {
            slot = raw_bridge_fast_packet_alloc_slot();
        }

        memset(slot, 0, sizeof(*slot));
        slot->in_use = 1u;
        slot->is_ble_to_n2k = is_ble_to_n2k;
        slot->source = source;
        slot->sequence_id = sequence_id;
        slot->pgn = pgn;
        slot->expected_len = expected_len;
        slot->next_frame_index = 1u;

        copy_len = (uint16_t)(frame->dlc - 2u);
        if (copy_len > expected_len) {
            copy_len = expected_len;
        }
        if (copy_len > 0u) {
            memcpy(slot->data, &frame->data[2], copy_len);
        }
        slot->data_len = copy_len;

#if RAW_BRIDGE_DEBUG_UART
        {
            char line[RAW_BRIDGE_DEBUG_LINE_LEN];
            (void)snprintf(line, sizeof(line),
                "%s FP_START src=%u pgn=%lu seq=%u expected=%u bytes_f0=%u\r\n",
                tag, (unsigned int)source, (unsigned long)pgn,
                (unsigned int)sequence_id, (unsigned int)expected_len,
                (unsigned int)copy_len);
            raw_bridge_uart_print_line(line);
        }
#endif

        if (slot->data_len >= slot->expected_len) {
            raw_bridge_fast_packet_handle_complete(tag, pgn, source, slot->data, slot->expected_len);
            raw_bridge_fast_packet_reset_slot(slot);
        }
        return;
    }

    slot = raw_bridge_fast_packet_find_slot(is_ble_to_n2k, source, pgn, sequence_id);
    if ((slot == 0) || (slot->next_frame_index != frame_index)) {
#if RAW_BRIDGE_DEBUG_UART
        {
            char line[RAW_BRIDGE_DEBUG_LINE_LEN];
            (void)snprintf(line, sizeof(line),
                "%s FP_DROP src=%u pgn=%lu seq=%u got_idx=%u expected_idx=%u\r\n",
                tag, (unsigned int)source, (unsigned long)pgn,
                (unsigned int)sequence_id, (unsigned int)frame_index,
                (slot != 0) ? (unsigned int)slot->next_frame_index : 99u);
            raw_bridge_uart_print_line(line);
        }
#endif
        return;
    }

    if (frame->dlc < 1u) {
        raw_bridge_fast_packet_reset_slot(slot);
        return;
    }

    remaining = (uint16_t)(slot->expected_len - slot->data_len);
    payload_offset = 1u;
    copy_len = (uint16_t)(frame->dlc - payload_offset);
    if (copy_len > remaining) {
        copy_len = remaining;
    }

    if ((copy_len > 0u) && ((uint16_t)(slot->data_len + copy_len) <= RAW_BRIDGE_FAST_PACKET_MAX_DATA_LEN)) {
        memcpy(&slot->data[slot->data_len], &frame->data[payload_offset], copy_len);
        slot->data_len = (uint16_t)(slot->data_len + copy_len);
    }
    slot->next_frame_index++;

#if RAW_BRIDGE_DEBUG_UART
    {
        char line[RAW_BRIDGE_DEBUG_LINE_LEN];
        (void)snprintf(line, sizeof(line),
            "%s FP_FRAME src=%u pgn=%lu seq=%u idx=%u bytes=%u/%u\r\n",
            tag, (unsigned int)source, (unsigned long)pgn,
            (unsigned int)sequence_id, (unsigned int)frame_index,
            (unsigned int)slot->data_len, (unsigned int)slot->expected_len);
        raw_bridge_uart_print_line(line);
    }
#endif

    if (slot->data_len >= slot->expected_len) {
        raw_bridge_fast_packet_handle_complete(tag, pgn, source, slot->data, slot->expected_len);
        raw_bridge_fast_packet_reset_slot(slot);
    }
}

static uint8_t raw_bridge_is_device_list_frame(
    const N2K_RawFrame_t *frame,
    uint32_t *pgn_out,
    uint32_t *requested_pgn_out
) {
    uint32_t pgn;
    uint32_t requested_pgn = UINT32_MAX;

    pgn = raw_bridge_get_pgn(frame);
    if (pgn_out != 0) {
        *pgn_out = pgn;
    }

    if (pgn == UINT32_MAX) {
        if (requested_pgn_out != 0) {
            *requested_pgn_out = requested_pgn;
        }
        return 0u;
    }

    if (pgn == N2K_PGN_ISO_REQUEST) {
        requested_pgn = raw_bridge_get_requested_pgn(frame);
        if (requested_pgn_out != 0) {
            *requested_pgn_out = requested_pgn;
        }
        return raw_bridge_is_device_list_pgn(requested_pgn);
    }

    if (requested_pgn_out != 0) {
        *requested_pgn_out = requested_pgn;
    }

    return raw_bridge_is_device_list_pgn(pgn);
}

static void raw_bridge_debug_device_list_frame(const char *tag, const N2K_RawFrame_t *frame) {
#if RAW_BRIDGE_DEBUG_UART
    uint32_t pgn;
    uint32_t requested_pgn;
    uint8_t is_ble_to_n2k;

    if (!raw_bridge_is_device_list_frame(frame, &pgn, &requested_pgn)) {
        return;
    }

    is_ble_to_n2k = (uint8_t)((strcmp(tag, "BLE->N2K") == 0) ? 1u : 0u);

    if (pgn == N2K_PGN_ISO_REQUEST) {
        raw_bridge_log_iso_request(tag, frame, requested_pgn);
        return;
    }

    if (pgn == N2K_PGN_ISO_ADDRESS_CLAIM) {
        raw_bridge_log_address_claim(tag, frame);
        return;
    }

    raw_bridge_debug_fast_packet(tag, frame, is_ble_to_n2k);
#else
    (void)tag;
    (void)frame;
#endif
}
#else
static void raw_bridge_debug_device_list_frame(const char *tag, const N2K_RawFrame_t *frame) {
    (void)tag;
    (void)frame;
}
#endif

static void raw_bridge_start_can(void) {
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000u;
    filter.FilterIdLow = 0x0000u;
    filter.FilterMaskIdHigh = 0x0000u;
    filter.FilterMaskIdLow = 0x0000u;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(s_hcan, &filter) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_Start(s_hcan) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(s_hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}

static void raw_bridge_decode_can_id(
    uint32_t can_id,
    uint8_t is_extended_id,
    uint8_t *priority_out,
    uint32_t *pgn_out,
    uint8_t *src_out,
    uint8_t *dst_out
) {
    uint8_t can_id_pf;
    uint8_t can_id_ps;
    uint8_t can_id_dp;
    uint8_t priority;
    uint32_t pgn;
    uint8_t src;
    uint8_t dst;

    if (is_extended_id == 0u) {
        if (priority_out != 0) {
            *priority_out = 0u;
        }
        if (pgn_out != 0) {
            *pgn_out = UINT32_MAX;
        }
        if (src_out != 0) {
            *src_out = 0u;
        }
        if (dst_out != 0) {
            *dst_out = 0xFFu;
        }
        return;
    }

    can_id_pf = (uint8_t)(can_id >> 16u);
    can_id_ps = (uint8_t)(can_id >> 8u);
    can_id_dp = (uint8_t)((can_id >> 24u) & 0x01u);

    src = (uint8_t)(can_id & 0xFFu);
    priority = (uint8_t)((can_id >> 26u) & 0x07u);

    if (can_id_pf < 240u) {
        dst = can_id_ps;
        pgn = (((uint32_t)can_id_dp) << 16u) | (((uint32_t)can_id_pf) << 8u);
    } else {
        dst = 0xFFu;
        pgn = (((uint32_t)can_id_dp) << 16u) | (((uint32_t)can_id_pf) << 8u) | (uint32_t)can_id_ps;
    }

    if (priority_out != 0) {
        *priority_out = priority;
    }
    if (pgn_out != 0) {
        *pgn_out = pgn;
    }
    if (src_out != 0) {
        *src_out = src;
    }
    if (dst_out != 0) {
        *dst_out = dst;
    }
}

static uint32_t raw_bridge_extract_pgn(uint32_t can_id, uint8_t is_extended_id) {
    uint32_t pgn = UINT32_MAX;
    raw_bridge_decode_can_id(can_id, is_extended_id, 0, &pgn, 0, 0);
    return pgn;
}

static uint8_t raw_bridge_extract_source(uint32_t can_id) {
    uint8_t src = 0u;
    raw_bridge_decode_can_id(can_id, 1u, 0, 0, &src, 0);
    return src;
}

static uint8_t raw_bridge_extract_destination(uint32_t can_id, uint8_t is_extended_id) {
    uint8_t dst = 0xFFu;
    raw_bridge_decode_can_id(can_id, is_extended_id, 0, 0, 0, &dst);
    return dst;
}

static void raw_bridge_log_event_push(const N2K_RawFrame_t *frame) {
    uint16_t log_next;
    N2K_RawBridgeRxEvent_t *slot;

    if ((frame == 0) || ((frame->flags & RAW_FRAME_FLAG_EXT_ID) == 0u)) {
        return;
    }
    log_next = (uint16_t)((s_log_event_head + 1u) % RAW_BRIDGE_LOG_EVENT_CAPACITY);
    if (log_next == s_log_event_tail) {
        return;
    }
    slot = &s_log_events[s_log_event_head];
    slot->can_id = frame->can_id & 0x1FFFFFFFu;
    raw_bridge_decode_can_id(frame->can_id, 1u,
        &slot->priority, &slot->pgn, &slot->src, &slot->dst);
    slot->timestamp_ms = frame->timestamp_ms;
    slot->dlc = frame->dlc;
    memset(slot->data, 0, sizeof(slot->data));
    if (frame->dlc > 0u) {
        memcpy(slot->data, frame->data, frame->dlc);
    }
    s_log_event_head = log_next;
}

static uint8_t raw_bridge_crc8(const uint8_t *data, uint16_t len) {
    uint16_t i;
    uint8_t crc = 0x00u;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

static void raw_bridge_spi_parse_reset(void) {
    s_spi_parser.state = SPI_PARSE_WAIT_SOF1;
    s_spi_parser.index = 0;
    s_spi_parser.expected_len = 0;
}

static uint8_t raw_bridge_spi_queue_push(const uint8_t *packet, uint16_t len) {
    uint8_t next_head;
    uint32_t primask;

    if ((packet == 0) || (len == 0u) || (len > RAW_BRIDGE_SPI_MAX_PACKET_LEN)) {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    next_head = (uint8_t)((s_spi_queue_head + 1u) % RAW_BRIDGE_SPI_QUEUE_CAPACITY);
    if (next_head == s_spi_queue_tail) {
        s_stats.spi_queue_overflow++;
        if (primask == 0u) {
            __enable_irq();
        }
        return 0u;
    }

    s_spi_queue[s_spi_queue_head].len = len;
    memcpy(s_spi_queue[s_spi_queue_head].bytes, packet, len);
    s_spi_queue_head = next_head;
    if (primask == 0u) {
        __enable_irq();
    }
    return 1u;
}

static uint8_t raw_bridge_spi_queue_pop(uint8_t *packet_out, uint16_t *len_out) {
    uint8_t has_packet = 0u;
    uint32_t primask;

    if ((packet_out == 0) || (len_out == 0)) {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_spi_queue_tail != s_spi_queue_head) {
        uint16_t len = s_spi_queue[s_spi_queue_tail].len;
        memcpy(packet_out, s_spi_queue[s_spi_queue_tail].bytes, len);
        *len_out = len;
        s_spi_queue_tail = (uint8_t)((s_spi_queue_tail + 1u) % RAW_BRIDGE_SPI_QUEUE_CAPACITY);
        has_packet = 1u;
    }
    if (primask == 0u) {
        __enable_irq();
    }

    return has_packet;
}


static uint8_t raw_bridge_can_transmit(const N2K_RawFrame_t *frame) {
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    if (frame->dlc > 8u) {
        s_stats.can_tx_errors++;
        return 0u;
    }

    memset(&tx_header, 0, sizeof(tx_header));
    tx_header.DLC = frame->dlc;
    tx_header.TransmitGlobalTime = DISABLE;
    tx_header.IDE = ((frame->flags & RAW_FRAME_FLAG_EXT_ID) != 0u) ? CAN_ID_EXT : CAN_ID_STD;
    tx_header.RTR = ((frame->flags & RAW_FRAME_FLAG_RTR) != 0u) ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    if (tx_header.IDE == CAN_ID_EXT) {
        tx_header.ExtId = frame->can_id & 0x1FFFFFFFu;
    } else {
        tx_header.StdId = frame->can_id & 0x7FFu;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(s_hcan) == 0u) {
        s_stats.can_tx_errors++;
        return 0u;
    }

    if (HAL_CAN_AddTxMessage(s_hcan, &tx_header, (uint8_t *)frame->data, &tx_mailbox) == HAL_OK) {
        s_stats.can_tx_frames++;
        if ((frame->flags & RAW_FRAME_FLAG_EXT_ID) != 0u) {
            s_pgn_debug.last_spi_to_can_pgn = raw_bridge_extract_pgn(frame->can_id, 1u);
            s_pgn_debug.last_spi_to_can_src = raw_bridge_extract_source(frame->can_id);
            s_pgn_debug.last_spi_to_can_dst = raw_bridge_extract_destination(frame->can_id, 1u);
            s_pgn_debug.spi_to_can_pgn_updates++;
        }
        raw_bridge_debug_device_list_frame("BLE->N2K", frame);
        return 1u;
    } else {
        s_stats.can_tx_errors++;
        return 0u;
    }
}

static void raw_bridge_handle_spi_packet(const uint8_t *packet, uint16_t packet_len) {
    N2K_RawFrame_t frame;
    uint8_t pkt_type;
    uint8_t crc;

    if (packet_len < 5u) {
        s_stats.spi_parse_errors++;
        return;
    }

    crc = raw_bridge_crc8(&packet[2], (uint16_t)(packet_len - 3u));
    if (crc != packet[packet_len - 1u]) {
        s_stats.spi_parse_errors++;
        return;
    }

    pkt_type = packet[2];
    if ((pkt_type == SPI_PACKET_TYPE_N2K_RX_FRAME) || (pkt_type == SPI_PACKET_TYPE_N2K_TX_FRAME)) {
        if (!SPI_Packet_ParseFramePacket(packet, packet_len, pkt_type, &frame)) {
            s_stats.spi_parse_errors++;
            return;
        }
        s_stats.spi_rx_packets++;

        if (pkt_type == SPI_PACKET_TYPE_N2K_TX_FRAME) {
            raw_bridge_can_transmit(&frame);
        }
        return;
    }

    if (pkt_type == SPI_PACKET_TYPE_STATUS) {
        s_stats.spi_rx_packets++;
        return;
    }

    if (pkt_type == SPI_PACKET_TYPE_DEVICE_LIST_REQUEST) {
        s_stats.spi_rx_packets++;
        DeviceListHandler_OnSpiRequest(
            (packet[3] > 0u) ? &packet[4] : 0,
            packet[3]
        );
        return;
    }

    s_stats.spi_parse_errors++;
}

static void raw_bridge_spi_parse_byte(uint8_t b) {
    switch (s_spi_parser.state) {
        case SPI_PARSE_WAIT_SOF1:
            if (b == SPI_PACKET_SOF1) {
                s_spi_parser.packet[0] = b;
                s_spi_parser.index = 1u;
                s_spi_parser.state = SPI_PARSE_WAIT_SOF2;
            }
            break;

        case SPI_PARSE_WAIT_SOF2:
            if (b == SPI_PACKET_SOF2) {
                s_spi_parser.packet[s_spi_parser.index++] = b;
                s_spi_parser.state = SPI_PARSE_READ_TYPE;
            } else if (b == SPI_PACKET_SOF1) {
                s_spi_parser.packet[0] = b;
                s_spi_parser.index = 1u;
            } else {
                raw_bridge_spi_parse_reset();
            }
            break;

        case SPI_PARSE_READ_TYPE:
            s_spi_parser.packet[s_spi_parser.index++] = b;
            s_spi_parser.state = SPI_PARSE_READ_LEN;
            break;

        case SPI_PARSE_READ_LEN:
            s_spi_parser.packet[s_spi_parser.index++] = b;
            s_spi_parser.expected_len = (uint16_t)(2u + 1u + 1u + (uint16_t)b + 1u);
            if ((s_spi_parser.expected_len > RAW_BRIDGE_SPI_MAX_PACKET_LEN) || (s_spi_parser.expected_len < 5u)) {
                s_stats.spi_parse_errors++;
                raw_bridge_spi_parse_reset();
            } else {
                s_spi_parser.state = SPI_PARSE_READ_PAYLOAD_CRC;
            }
            break;

        case SPI_PARSE_READ_PAYLOAD_CRC:
            s_spi_parser.packet[s_spi_parser.index++] = b;
            if (s_spi_parser.index >= s_spi_parser.expected_len) {
                raw_bridge_handle_spi_packet(s_spi_parser.packet, s_spi_parser.expected_len);
                raw_bridge_spi_parse_reset();
            }
            break;

        default:
            raw_bridge_spi_parse_reset();
            break;
    }
}

static uint8_t raw_bridge_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len) {
    HAL_GPIO_WritePin(CS_EXT_GPIO_Port, CS_EXT_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(s_hspi, (uint8_t *)tx, rx, len, 100u) == HAL_OK) {
        uint16_t i;
        for (i = 0; i < len; i++) {
            raw_bridge_spi_parse_byte(rx[i]);
        }
        HAL_GPIO_WritePin(CS_EXT_GPIO_Port, CS_EXT_Pin, GPIO_PIN_SET);
        return 1u;
    }
    HAL_GPIO_WritePin(CS_EXT_GPIO_Port, CS_EXT_Pin, GPIO_PIN_SET);
    return 0u;
}

static void raw_bridge_spi_poll(void) {
    uint8_t tx_dummy[RAW_BRIDGE_SPI_POLL_BYTES];
    uint8_t rx_dummy[RAW_BRIDGE_SPI_POLL_BYTES];
    memset(tx_dummy, 0xFF, sizeof(tx_dummy));
    (void)raw_bridge_spi_transfer(tx_dummy, rx_dummy, (uint16_t)sizeof(tx_dummy));
}

/* Always-on fast-packet assembler for N2K identity PGNs.
 * Uses dedicated slots (s_id_fp_slots) independent of the debug path so that
 * ProductInfo / ConfigInfo / PGNList are reassembled even when
 * RAW_BRIDGE_DEBUG_UART=0.  Completed packets are pushed to the assembled
 * event FIFO for consumption by DeviceListHandler_OnAssembledEvent(). */
static void raw_bridge_identity_frame_process(const N2K_RawFrame_t *frame) {
    RawBridgeFastPacketSlot_t *slot;
    uint32_t pgn;
    uint8_t src;
    uint8_t seq;
    uint8_t idx;
    uint8_t i;
    uint16_t copy_len;
    uint16_t remaining;

    if ((frame == 0) || (frame->dlc == 0u)) {
        return;
    }
    if ((frame->flags & RAW_FRAME_FLAG_EXT_ID) == 0u) {
        return;
    }

    pgn = raw_bridge_extract_pgn(frame->can_id, 1u);
    if ((pgn != N2K_PGN_PRODUCT_INFORMATION) &&
        (pgn != N2K_PGN_CONFIGURATION_INFORMATION) &&
        (pgn != N2K_PGN_PGN_LIST)) {
        return;
    }

    src = raw_bridge_extract_source(frame->can_id);
    seq = (uint8_t)(frame->data[0] >> 5u);
    idx = (uint8_t)(frame->data[0] & 0x1Fu);

    if (idx == 0u) {
        /* First frame of fast-packet sequence. */
        uint16_t expected_len;

        if (frame->dlc < 2u) {
            return;
        }
        expected_len = frame->data[1];
        if ((expected_len == 0u) || (expected_len > RAW_BRIDGE_FAST_PACKET_MAX_DATA_LEN)) {
            return;
        }

        /* Find existing slot for this source/pgn/seq or allocate a free one. */
        slot = 0;
        for (i = 0u; i < RAW_BRIDGE_ID_FP_SLOTS; i++) {
            if ((s_id_fp_slots[i].in_use != 0u) &&
                (s_id_fp_slots[i].source == src) &&
                (s_id_fp_slots[i].pgn == pgn) &&
                (s_id_fp_slots[i].sequence_id == seq)) {
                slot = &s_id_fp_slots[i];
                break;
            }
        }
        if (slot == 0) {
            for (i = 0u; i < RAW_BRIDGE_ID_FP_SLOTS; i++) {
                if (s_id_fp_slots[i].in_use == 0u) {
                    slot = &s_id_fp_slots[i];
                    break;
                }
            }
        }
        if (slot == 0) {
            slot = &s_id_fp_slots[0];  /* Evict oldest slot on overflow. */
        }

        memset(slot, 0, sizeof(*slot));
        slot->in_use = 1u;
        slot->source = src;
        slot->sequence_id = seq;
        slot->pgn = pgn;
        slot->expected_len = expected_len;
        slot->next_frame_index = 1u;

        copy_len = (uint16_t)(frame->dlc - 2u);
        if (copy_len > expected_len) {
            copy_len = expected_len;
        }
        if (copy_len > 0u) {
            memcpy(slot->data, &frame->data[2], copy_len);
        }
        slot->data_len = copy_len;
    } else {
        /* Continuation frame. */
        slot = 0;
        for (i = 0u; i < RAW_BRIDGE_ID_FP_SLOTS; i++) {
            if ((s_id_fp_slots[i].in_use != 0u) &&
                (s_id_fp_slots[i].source == src) &&
                (s_id_fp_slots[i].pgn == pgn) &&
                (s_id_fp_slots[i].sequence_id == seq)) {
                slot = &s_id_fp_slots[i];
                break;
            }
        }
        if ((slot == 0) || (slot->next_frame_index != idx)) {
            return;  /* Out-of-order or unknown sequence; discard. */
        }
        if (frame->dlc < 1u) {
            memset(slot, 0, sizeof(*slot));
            return;
        }

        remaining = (uint16_t)(slot->expected_len - slot->data_len);
        copy_len = (uint16_t)(frame->dlc - 1u);
        if (copy_len > remaining) {
            copy_len = remaining;
        }
        if ((copy_len > 0u) &&
            ((uint16_t)(slot->data_len + copy_len) <= RAW_BRIDGE_FAST_PACKET_MAX_DATA_LEN)) {
            memcpy(&slot->data[slot->data_len], &frame->data[1], copy_len);
            slot->data_len = (uint16_t)(slot->data_len + copy_len);
        }
        slot->next_frame_index++;
    }

    if (slot->data_len >= slot->expected_len) {
        /* Assembly complete: push to assembled event FIFO. */
        uint8_t next = (uint8_t)((s_assembled_head + 1u) % RAW_BRIDGE_ASSEMBLED_QUEUE_LEN);
        if (next != s_assembled_tail) {
            N2K_RawBridgeAssembledEvent_t *ev = &s_assembled_events[s_assembled_head];
            ev->pgn = slot->pgn;
            ev->src = slot->source;
            ev->len = slot->expected_len;
            memcpy(ev->data, slot->data, slot->expected_len);
            s_assembled_head = next;
        }
        memset(slot, 0, sizeof(*slot));
    }
}

void N2K_RawBridge_Init(CAN_HandleTypeDef *hcan, SPI_HandleTypeDef *hspi) {
    s_hcan = hcan;
    s_hspi = hspi;
    memset((void *)&s_stats, 0, sizeof(s_stats));
    memset((void *)&s_pgn_debug, 0, sizeof(s_pgn_debug));
    s_pgn_debug.last_can_rx_pgn = UINT32_MAX;
    s_pgn_debug.last_spi_to_can_pgn = UINT32_MAX;
    RawRingBuffer_Init(&s_ring, s_ring_storage, RAW_BRIDGE_RING_CAPACITY);
    memset(s_log_events, 0, sizeof(s_log_events));
    s_log_event_head = 0u;
    s_log_event_tail = 0u;
    memset(s_spi_queue, 0, sizeof(s_spi_queue));
    s_spi_queue_head = 0u;
    s_spi_queue_tail = 0u;
    raw_bridge_spi_parse_reset();

    HAL_GPIO_WritePin(CS_EXT_GPIO_Port, CS_EXT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_SPI_GPIO_Port, EN_SPI_Pin, GPIO_PIN_SET);

    raw_bridge_start_can();
}

void N2K_RawBridge_Process(void) {
    N2K_RawFrame_t frame;
    uint8_t packet[RAW_BRIDGE_SPI_MAX_PACKET_LEN];
    uint8_t spi_rx[RAW_BRIDGE_SPI_MAX_PACKET_LEN];
    size_t packet_len = 0;
    uint16_t queued_len = 0u;
    uint8_t frames_sent = 0u;
    char line[96];

    while (raw_bridge_spi_queue_pop(packet, &queued_len) != 0u) {
        uint8_t sent = 0u;
        uint8_t attempts = 0u;

        if (frames_sent > 0u) {
            HAL_Delay(1u);
        }

        while (attempts < 3u) {
            attempts++;
            if (raw_bridge_spi_transfer(packet, spi_rx, queued_len) != 0u) {
                sent = 1u;
                s_stats.spi_tx_frames++;
                HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
                break;
            }
            HAL_Delay(1u);
        }

        if ((packet[2] == SPI_PACKET_TYPE_DEVICE_LIST) ||
            (packet[2] == SPI_PACKET_TYPE_DEVICE_LIST_REQUEST)) {
            (void)snprintf(line,
                           sizeof(line),
                           "[SPIQ] type=0x%02X len=%u result=%s tries=%u\r\n",
                           (unsigned)packet[2],
                           (unsigned)queued_len,
                           sent != 0u ? "ok" : "fail",
                           (unsigned)attempts);
            HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)strlen(line), 100u);
        }
        frames_sent++;
    }

    /* Drain the ring buffer FIRST, before any poll.
     * The ESP32 SPI slave needs ~5-20 us to process a completed transaction
     * and re-queue for the next one.  If we polled first, the real frame
     * packet would follow within ~500 ns (a few STM32 instructions) — well
     * inside the ESP32 re-queue blind spot.  By skipping the poll when there
     * is data, the frame is sent while the ESP32 is already idle in
     * spi_slave_transmit from the previous cycle.
     *
     * Limit to 8 frames per call to keep the main loop responsive. */
    while ((frames_sent < 8u) && RawRingBuffer_Pop(&s_ring, &frame)) {
        /* Track new CAN sources and assemble identity fast-packets for
         * DeviceListHandler — always, regardless of SPI forwarding. */
        if ((frame.flags & RAW_FRAME_FLAG_EXT_ID) != 0u) {
            uint8_t src = raw_bridge_extract_source(frame.can_id);
            uint8_t byte_idx = (uint8_t)(src >> 3u);
            uint8_t bit_mask = (uint8_t)(1u << (src & 0x07u));
            if ((s_seen_sources[byte_idx] & bit_mask) == 0u) {
                uint8_t nxt = (uint8_t)((s_new_source_head + 1u) % RAW_BRIDGE_NEW_SOURCE_QUEUE_LEN);
                s_seen_sources[byte_idx] |= bit_mask;
                if (nxt != s_new_source_tail) {
                    s_new_source_queue[s_new_source_head] = src;
                    s_new_source_head = nxt;
                }
            }
        }
        raw_bridge_identity_frame_process(&frame);

        if (!SPI_Packet_BuildFramePacket(SPI_PACKET_TYPE_N2K_RX_FRAME, &frame, packet, sizeof(packet), &packet_len)) {
            continue;
        }

        /* For back-to-back burst frames (e.g. fast-packet metadata responses)
         * give the ESP32 time to re-queue between consecutive CS pulses. */
        if (frames_sent > 0u) {
            HAL_Delay(1u);
        }

        if (raw_bridge_spi_transfer(packet, spi_rx, (uint16_t)packet_len) != 0u) {
            s_stats.spi_tx_frames++;
            raw_bridge_debug_device_list_frame("N2K->BLE", &frame);
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        }
        frames_sent++;
    }

    /* Always poll so we read any pending ESP32-to-STM32 command (such as
     * a device-list request) even when outbound CAN frames were sent.
     * The SPI transfer for outbound frames also reads MISO bytes, but
     * the device-list request might arrive in a later ESP32 DMA slot
     * that only a dedicated poll transaction can pick up. */
    if (frames_sent > 0u) {
        HAL_Delay(1u);
    }
    raw_bridge_spi_poll();
}

N2K_RawBridgeStats_t N2K_RawBridge_GetStats(void) {
    N2K_RawBridgeStats_t copy;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    copy = s_stats;
    if (primask == 0u) {
        __enable_irq();
    }
    return copy;
}

N2K_RawBridgePgnDebug_t N2K_RawBridge_GetPgnDebug(void) {
    N2K_RawBridgePgnDebug_t copy;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    copy = s_pgn_debug;
    if (primask == 0u) {
        __enable_irq();
    }
    return copy;
}

uint8_t N2K_RawBridge_PopLogEvent(N2K_RawBridgeRxEvent_t *event_out) {
    uint8_t has_event = 0u;
    uint32_t primask;

    if (event_out == 0) {
        return 0u;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (s_log_event_tail != s_log_event_head) {
        *event_out = s_log_events[s_log_event_tail];
        s_log_event_tail = (uint16_t)((s_log_event_tail + 1u) % RAW_BRIDGE_LOG_EVENT_CAPACITY);
        has_event = 1u;
    }
    if (primask == 0u) {
        __enable_irq();
    }
    return has_event;
}

    uint8_t N2K_RawBridge_SendIsoRequest(uint8_t src, uint8_t dst, uint32_t requested_pgn) {
        N2K_RawFrame_t frame;
        uint32_t can_id;

        /* PGN 59904 (ISO Request): prio=6, PF=0xEA, PS=dst, DP=0 */
        if (s_hcan == 0) {
            return 0u;
        }

        memset(&frame, 0, sizeof(frame));
        can_id = ((uint32_t)6u << 26u) |
                 ((uint32_t)0u << 24u) |
                 ((uint32_t)0xEAu << 16u) |
                 ((uint32_t)dst << 8u) |
                 (uint32_t)src;

        frame.can_id = can_id;
        frame.flags = RAW_FRAME_FLAG_EXT_ID;
        frame.dlc = 3u;
        frame.data[0] = (uint8_t)(requested_pgn & 0xFFu);
        frame.data[1] = (uint8_t)((requested_pgn >> 8u) & 0xFFu);
        frame.data[2] = (uint8_t)((requested_pgn >> 16u) & 0xFFu);

        return raw_bridge_can_transmit(&frame);
    }

    uint8_t N2K_RawBridge_QueueSpiPacket(uint8_t pkt_type, const uint8_t *payload, uint8_t payload_len) {
        uint8_t packet[RAW_BRIDGE_SPI_MAX_PACKET_LEN];
        size_t packet_len = 0u;

        if (!SPI_Packet_BuildCustomPacket(pkt_type,
                                          payload,
                                          payload_len,
                                          packet,
                                          sizeof(packet),
                                          &packet_len)) {
            return 0u;
        }

        if (packet_len > 0xFFFFu) {
            return 0u;
        }

        return raw_bridge_spi_queue_push(packet, (uint16_t)packet_len);
    }

uint8_t N2K_RawBridge_PopAssembledEvent(N2K_RawBridgeAssembledEvent_t *event_out) {
    if (event_out == 0) {
        return 0u;
    }
    if (s_assembled_tail == s_assembled_head) {
        return 0u;
    }
    *event_out = s_assembled_events[s_assembled_tail];
    s_assembled_tail = (uint8_t)((s_assembled_tail + 1u) % RAW_BRIDGE_ASSEMBLED_QUEUE_LEN);
    return 1u;
}

void N2K_RawBridge_ResetSeenSources(void) {
    memset(s_seen_sources, 0, sizeof(s_seen_sources));
    s_new_source_head = 0u;
    s_new_source_tail = 0u;
}

uint8_t N2K_RawBridge_PopNewSource(uint8_t *src_out) {
    if (src_out == 0) {
        return 0u;
    }
    if (s_new_source_tail == s_new_source_head) {
        return 0u;
    }
    *src_out = s_new_source_queue[s_new_source_tail];
    s_new_source_tail = (uint8_t)((s_new_source_tail + 1u) % RAW_BRIDGE_NEW_SOURCE_QUEUE_LEN);
    return 1u;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    if (hcan != s_hcan) {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0u) {
        N2K_RawFrame_t frame;
        uint8_t dlc;

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK) {
            break;
        }

        memset(&frame, 0, sizeof(frame));
        s_stats.can_rx_frames++;
        frame.timestamp_ms = HAL_GetTick();
        frame.dlc = rx_header.DLC;

        if (rx_header.IDE == CAN_ID_EXT) {
            frame.can_id = rx_header.ExtId & 0x1FFFFFFFu;
            frame.flags |= RAW_FRAME_FLAG_EXT_ID;
        } else {
            frame.can_id = rx_header.StdId & 0x7FFu;
        }
        if (rx_header.RTR == CAN_RTR_REMOTE) {
            frame.flags |= RAW_FRAME_FLAG_RTR;
        }
        frame.flags &= (uint8_t)~RAW_FRAME_FLAG_DIRECTION;

        dlc = frame.dlc;
        if (dlc > 8u) {
            s_stats.spi_parse_errors++;
            dlc = 8u;
            frame.dlc = 8u;
        }
        if (dlc > 0u) {
            memcpy(frame.data, data, dlc);
        }

        if ((frame.flags & RAW_FRAME_FLAG_EXT_ID) != 0u) {
            s_pgn_debug.last_can_rx_pgn = raw_bridge_extract_pgn(frame.can_id, 1u);
            s_pgn_debug.last_can_rx_src = raw_bridge_extract_source(frame.can_id);
            s_pgn_debug.last_can_rx_dst = raw_bridge_extract_destination(frame.can_id, 1u);
            s_pgn_debug.last_can_rx_dlc = dlc;
            memset((void *)s_pgn_debug.last_can_rx_data, 0, sizeof(s_pgn_debug.last_can_rx_data));
            if (dlc > 0u) {
                memcpy((void *)s_pgn_debug.last_can_rx_data, frame.data, dlc);
            }
            s_pgn_debug.can_rx_pgn_updates++;
            raw_bridge_log_event_push(&frame);
        }

        if (!RawRingBuffer_Push(&s_ring, &frame)) {
            s_stats.can_rx_overflow++;
        } else {
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        }
    }
}
