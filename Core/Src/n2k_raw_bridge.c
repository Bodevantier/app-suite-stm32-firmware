#include "n2k_raw_bridge.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "raw_ring_buffer.h"
#include "spi_packet.h"
#include "usart.h"

#define RAW_BRIDGE_RING_CAPACITY 128u
#define RAW_FRAME_FLAG_EXT_ID 0x01u
#define RAW_FRAME_FLAG_RTR 0x02u
#define RAW_FRAME_FLAG_DIRECTION 0x04u
#define RAW_BRIDGE_SPI_POLL_BYTES SPI_PACKET_TOTAL_LEN
#define RAW_BRIDGE_SPI_MAX_PACKET_LEN 260u
#define RAW_BRIDGE_DEBUG_UART 0u
#define RAW_BRIDGE_DEBUG_LINE_LEN 320u
#define RAW_BRIDGE_FAST_PACKET_SLOTS 6u
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

        if (slot->data_len >= slot->expected_len) {
            raw_bridge_fast_packet_handle_complete(tag, pgn, source, slot->data, slot->expected_len);
            raw_bridge_fast_packet_reset_slot(slot);
        }
        return;
    }

    slot = raw_bridge_fast_packet_find_slot(is_ble_to_n2k, source, pgn, sequence_id);
    if ((slot == 0) || (slot->next_frame_index != frame_index)) {
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

static void raw_bridge_can_transmit(const N2K_RawFrame_t *frame) {
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    if (frame->dlc > 8u) {
        s_stats.can_tx_errors++;
        return;
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
        return;
    }

    if (HAL_CAN_AddTxMessage(s_hcan, &tx_header, (uint8_t *)frame->data, &tx_mailbox) == HAL_OK) {
        s_stats.can_tx_frames++;
        raw_bridge_debug_device_list_frame("BLE->N2K", frame);
    } else {
        s_stats.can_tx_errors++;
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

void N2K_RawBridge_Init(CAN_HandleTypeDef *hcan, SPI_HandleTypeDef *hspi) {
    s_hcan = hcan;
    s_hspi = hspi;
    memset((void *)&s_stats, 0, sizeof(s_stats));

    RawRingBuffer_Init(&s_ring, s_ring_storage, RAW_BRIDGE_RING_CAPACITY);
    raw_bridge_spi_parse_reset();

    HAL_GPIO_WritePin(CS_EXT_GPIO_Port, CS_EXT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_SPI_GPIO_Port, EN_SPI_Pin, GPIO_PIN_SET);

    raw_bridge_start_can();
}

void N2K_RawBridge_Process(void) {
    N2K_RawFrame_t frame;
    uint8_t packet[SPI_PACKET_TOTAL_LEN];
    uint8_t spi_rx[SPI_PACKET_TOTAL_LEN];
    size_t packet_len = 0;
    uint8_t frames_sent = 0u;

    /* Drain the ring buffer FIRST, before any poll.
     * The ESP32 SPI slave needs ~5-20 us to process a completed transaction
     * and re-queue for the next one.  If we polled first, the real frame
     * packet would follow within ~500 ns (a few STM32 instructions) — well
     * inside the ESP32 re-queue blind spot.  By skipping the poll when there
     * is data, the frame is sent while the ESP32 is already idle in
     * spi_slave_transmit from the previous cycle. */
    while (RawRingBuffer_Pop(&s_ring, &frame)) {
        if (!SPI_Packet_BuildFramePacket(SPI_PACKET_TYPE_N2K_RX_FRAME, &frame, packet, sizeof(packet), &packet_len)) {
            continue;
        }

        /* For back-to-back burst frames (e.g. fast-packet metadata responses)
         * give the ESP32 time to re-queue between consecutive CS pulses. */
        if (frames_sent > 0u) {
            HAL_Delay(5u);
        }

        if (raw_bridge_spi_transfer(packet, spi_rx, (uint16_t)packet_len) != 0u) {
            s_stats.spi_tx_frames++;
            raw_bridge_debug_device_list_frame("N2K->BLE", &frame);
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        }
        frames_sent++;
    }

    /* Only poll (for ESP32 -> CAN TX direction) when there were no frames to
     * send.  This keeps the bidirectional link alive without creating a
     * back-to-back collision with an outbound frame send. */
    if (frames_sent == 0u) {
        raw_bridge_spi_poll();
    }
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

        if (!RawRingBuffer_Push(&s_ring, &frame)) {
            s_stats.can_rx_overflow++;
        } else {
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        }
    }
}
