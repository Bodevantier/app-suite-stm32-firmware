#include "devicelist_handler.h"

#include <stdio.h>
#include <string.h>
#include <new>

#ifndef N2kMaxBusDevices
#define N2kMaxBusDevices 254
#endif
#include "main.h"
#include "spi_packet.h"
#include "usart.h"

namespace {
/* Display/identity address used in BLE device-list snapshots so the Flutter
   app sees the bridge as a labelled gateway entry. The bridge does NOT run an
   address-claim for this source — it is purely a label. */
constexpr uint8_t kOwnN2kSource = 15u;
/* Source address used when the bridge transmits ISO Requests (PGN 59904).
   Per ISO 11783 / NMEA 2000 §3.5, a node may only transmit on a source it has
   claimed via PGN 60928. Since this bridge does NOT participate in address
   claim, all bridge-originated requests use the NULL source 0xFE
   ("anonymous requester"), which is the standard compliant lurker pattern. */
constexpr uint8_t kIsoRequestSrc = 0xFEu;
constexpr uint8_t kIsoDstGlobal = 255u;
constexpr uint32_t kPgnIsoRequest = 59904u;
constexpr uint32_t kPgnAddressClaim = 60928u;
constexpr uint32_t kPgnProductInfo = 126996u;
constexpr uint32_t kPgnConfigInfo = 126998u;
constexpr uint32_t kPgnPgnList = 126464u;

constexpr uint32_t kRefreshQuietWindowMs = 300u;   /* was 500ms - faster quiet gate */
constexpr uint32_t kRefreshTimeoutMs = 3000u;      /* was 7000ms - max wait for responses */
constexpr uint32_t kDeviceOnlineMs = 10000u;
constexpr uint32_t kSnapshotRetryBackoffMs = 40u;
constexpr uint8_t kSnapshotMaxRetries = 5u;

/* Device-list canonical transport format (aligned with reference project). */
constexpr uint8_t kDevListFormatVersion = 1u;
constexpr uint8_t kDevListMsgBegin = 1u;
constexpr uint8_t kDevListMsgDevice = 2u;
constexpr uint8_t kDevListMsgEnd = 3u;

constexpr uint8_t kFieldModelId = 1u;
constexpr uint8_t kFieldSoftwareVersion = 2u;
constexpr uint8_t kFieldModelVersion = 3u;
constexpr uint8_t kFieldSerialCode = 4u;
constexpr uint8_t kFieldManufacturerText = 5u;
constexpr uint8_t kFieldInstallation1 = 6u;
constexpr uint8_t kFieldInstallation2 = 7u;
constexpr uint8_t kFieldTxPgnList = 8u;
constexpr uint8_t kFieldRxPgnList = 9u;

constexpr uint16_t kFlagOnline = 0x0001u;
constexpr uint16_t kFlagHasAddressClaim = 0x0002u;
constexpr uint16_t kFlagHasProductInfo = 0x0004u;
constexpr uint16_t kFlagHasConfigInfo = 0x0008u;
constexpr uint16_t kFlagHasTxPgnList = 0x0010u;
constexpr uint16_t kFlagHasRxPgnList = 0x0020u;

constexpr uint8_t kMaskAddressClaim = 0x01u;
constexpr uint8_t kMaskProductInfo = 0x02u;
constexpr uint8_t kMaskConfigInfo = 0x04u;
constexpr uint8_t kMaskTxPgnList = 0x08u;
constexpr uint8_t kMaskRxPgnList = 0x10u;

constexpr uint8_t kRequestProductInfo = 0x01u;
constexpr uint8_t kRequestConfigInfo = 0x02u;
constexpr uint8_t kRequestPgnList = 0x04u;

constexpr uint16_t kIdentitySlots = N2kMaxBusDevices;
constexpr uint8_t kTextMaxLen = 72u;
constexpr uint8_t kPgnMaxPerDirection = 64u;

struct DeviceInfo_t {
  uint8_t used;
  uint8_t source;
  uint8_t seen_mask;
  uint8_t request_mask;
  uint32_t last_seen_ms;

  uint64_t name;
  uint32_t unique;
  uint16_t manufacturer;
  uint16_t product;

  uint8_t device_function;
  uint8_t device_class;
  uint8_t device_instance;
  uint8_t system_instance;
  uint8_t industry_group;

  char *model_id;
  char *sw_version;
  char *model_version;
  char *serial;
  char *manufacturer_text;
  char *installation1;
  char *installation2;

  uint8_t tx_pgn_count;
  uint8_t rx_pgn_count;
  uint32_t *tx_pgn;
  uint32_t *rx_pgn;
};

uint8_t s_refresh_pending = 0u;
uint16_t s_request_id = 1u;
uint16_t s_active_request_id = 0u;
uint32_t s_refresh_started_ms = 0u;
uint32_t s_refresh_last_activity_ms = 0u;
DeviceInfo_t *s_devices[kIdentitySlots] = {};

uint8_t s_snapshot_retry_pending = 0u;
uint16_t s_snapshot_retry_request_id = 0u;
uint8_t s_snapshot_retry_attempt = 0u;
uint32_t s_snapshot_retry_at_ms = 0u;
uint32_t s_devlist_queue_overflow_count = 0u;
uint32_t s_devlist_retry_count = 0u;
uint32_t s_devlist_retry_giveup_count = 0u;
uint32_t s_devlist_alloc_fail_count = 0u;

void handler_uart_print(const char *msg) {
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)strlen(msg), 100u);
}

void write_u16_le(uint8_t *dst, uint16_t value) {
  dst[0] = (uint8_t)(value & 0xFFu);
  dst[1] = (uint8_t)((value >> 8u) & 0xFFu);
}

void write_u32_le(uint8_t *dst, uint32_t value) {
  dst[0] = (uint8_t)(value & 0xFFu);
  dst[1] = (uint8_t)((value >> 8u) & 0xFFu);
  dst[2] = (uint8_t)((value >> 16u) & 0xFFu);
  dst[3] = (uint8_t)((value >> 24u) & 0xFFu);
}

uint16_t read_u16_le(const uint8_t *src) {
  return (uint16_t)src[0] | ((uint16_t)src[1] << 8u);
}

uint32_t read_u24_le(const uint8_t *src) {
  return ((uint32_t)src[0]) |
         ((uint32_t)src[1] << 8u) |
         ((uint32_t)src[2] << 16u);
}

uint64_t read_u64_le(const uint8_t *src) {
  uint64_t value = 0u;
  uint8_t i;
  for (i = 0u; i < 8u; i++) {
    value |= ((uint64_t)src[i]) << (8u * i);
  }
  return value;
}

void write_common_header(uint8_t *dst, uint8_t message_type, uint16_t request_id, uint32_t snapshot_ms) {
  dst[0] = kDevListFormatVersion;
  dst[1] = message_type;
  write_u16_le(&dst[2], request_id);
  write_u32_le(&dst[4], snapshot_ms);
}

uint8_t queue_devlist_payload(const uint8_t *payload, uint8_t payload_len) {
  uint8_t ok = N2K_RawBridge_QueueSpiPacket(SPI_PACKET_TYPE_DEVICE_LIST, payload, payload_len);
  if (ok == 0u) {
    s_devlist_queue_overflow_count++;
  }
  return ok;
}

void free_device(DeviceInfo_t *device) {
  if (device == nullptr) {
    return;
  }

  delete[] device->model_id;
  delete[] device->sw_version;
  delete[] device->model_version;
  delete[] device->serial;
  delete[] device->manufacturer_text;
  delete[] device->installation1;
  delete[] device->installation2;
  delete[] device->tx_pgn;
  delete[] device->rx_pgn;
  delete device;
}

void reset_refresh_tracking(void) {
  for (uint16_t i = 0u; i < kIdentitySlots; i++) {
    if (s_devices[i] != nullptr) {
      free_device(s_devices[i]);
      s_devices[i] = nullptr;
    }
  }
}

DeviceInfo_t *find_or_alloc_device(uint8_t source) {
  DeviceInfo_t *device;

  if (source >= kIdentitySlots) {
    return nullptr;
  }

  device = s_devices[source];
  if (device != nullptr) {
    return device;
  }

  device = new (std::nothrow) DeviceInfo_t();
  if (device == nullptr) {
    s_devlist_alloc_fail_count++;
    return nullptr;
  }

  (void)memset(device, 0, sizeof(*device));
  device->used = 1u;
  device->source = source;
  device->last_seen_ms = HAL_GetTick();
  s_devices[source] = device;
  return device;
}

uint8_t count_seen_devices(void) {
  uint8_t count = 0u;
  for (uint16_t i = 0u; i < kIdentitySlots; i++) {
    if ((s_devices[i] != nullptr) && (s_devices[i]->used != 0u)) {
      count++;
    }
  }
  return count;
}

uint8_t count_ready_devices(void) {
  uint8_t count = 0u;
  for (uint16_t i = 0u; i < kIdentitySlots; i++) {
    if ((s_devices[i] != nullptr) &&
        (s_devices[i]->used != 0u) &&
        ((s_devices[i]->seen_mask & (kMaskTxPgnList | kMaskRxPgnList)) ==
         (kMaskTxPgnList | kMaskRxPgnList))) {
      count++;
    }
  }
  return count;
}

uint8_t all_tracked_devices_have_pgn_lists(void) {
  uint8_t found_remote_device = 0u;

  for (uint16_t i = 0u; i < kIdentitySlots; i++) {
    if ((s_devices[i] == nullptr) || (s_devices[i]->used == 0u)) {
      continue;
    }

    found_remote_device = 1u;
    if ((s_devices[i]->seen_mask & (kMaskTxPgnList | kMaskRxPgnList)) !=
        (kMaskTxPgnList | kMaskRxPgnList)) {
      return 0u;
    }
  }

  return found_remote_device;
}

void copy_n2k_padded_text(char *dst, size_t dst_size, const uint8_t *src, size_t src_len) {
  size_t i;
  if ((dst == nullptr) || (dst_size == 0u)) {
    return;
  }

  for (i = 0u; (i + 1u) < dst_size && i < src_len; i++) {
    uint8_t c = src[i];
    if ((c == 0x00u) || (c == 0xFFu)) {
      break;
    }
    dst[i] = (char)c;
  }
  dst[i] = '\0';
}

char *clone_text(const char *src) {
  size_t len;
  char *copy;

  if ((src == nullptr) || (src[0] == '\0')) {
    return nullptr;
  }

  len = strlen(src);
  copy = new (std::nothrow) char[len + 1u];
  if (copy == nullptr) {
    s_devlist_alloc_fail_count++;
    return nullptr;
  }

  (void)memcpy(copy, src, len + 1u);
  return copy;
}

void replace_text_field(char *&dst, const char *src) {
  char *copy = clone_text(src);

  if ((src != nullptr) && (src[0] != '\0') && (copy == nullptr)) {
    return;
  }

  delete[] dst;
  dst = copy;
}

void copy_n2k_text_field(char *&dst, const uint8_t *src, size_t src_len) {
  char temp[kTextMaxLen];

  copy_n2k_padded_text(temp, sizeof(temp), src, src_len);
  replace_text_field(dst, temp);
}

void replace_pgn_list(uint32_t *&dst, uint8_t &dst_count, const uint8_t *src, uint8_t count) {
  uint32_t *copy = nullptr;

  if ((src != nullptr) && (count > 0u)) {
    copy = new (std::nothrow) uint32_t[count];
    if (copy == nullptr) {
      s_devlist_alloc_fail_count++;
      return;
    }
    for (uint8_t i = 0u; i < count; i++) {
      copy[i] = read_u24_le(&src[1u + (uint16_t)i * 3u]);
    }
  }

  delete[] dst;
  dst = copy;
  dst_count = count;
}

uint8_t parse_var_string(const uint8_t *payload,
                         uint16_t payload_len,
                         uint16_t *index,
                         char *dst,
                         size_t dst_size) {
  uint16_t off;
  uint8_t field_len;
  uint8_t field_type;
  uint8_t text_len;

  if ((payload == nullptr) || (index == nullptr) || (dst == nullptr)) {
    return 0u;
  }

  off = *index;
  if ((uint16_t)(off + 2u) > payload_len) {
    return 0u;
  }

  field_len = payload[off++];
  field_type = payload[off++];
  if ((field_len < 2u) || (field_type != 0x01u)) {
    return 0u;
  }

  text_len = (uint8_t)(field_len - 2u);
  if ((uint16_t)(off + text_len) > payload_len) {
    return 0u;
  }

  copy_n2k_padded_text(dst, dst_size, &payload[off], text_len);
  *index = (uint16_t)(off + text_len);
  return 1u;
}

uint8_t append_tlv_text(uint8_t *msg,
                        uint16_t *offset,
                        uint16_t max_len,
                        uint8_t type,
                        const char *text,
                        uint8_t *field_count) {
  uint16_t len;
  if ((text == nullptr) || (text[0] == '\0')) {
    return 1u;
  }

  len = (uint16_t)strlen(text);
  if ((uint16_t)(*offset + 3u + len) > max_len) {
    return 0u;
  }

  msg[(*offset)++] = type;
  write_u16_le(&msg[*offset], len);
  *offset = (uint16_t)(*offset + 2u);
  (void)memcpy(&msg[*offset], text, len);
  *offset = (uint16_t)(*offset + len);
  (*field_count)++;
  return 1u;
}

uint8_t append_tlv_pgn_list(uint8_t *msg,
                            uint16_t *offset,
                            uint16_t max_len,
                            uint8_t type,
                            const uint32_t *pgns,
                            uint8_t pgn_count,
                            uint8_t *field_count) {
  uint16_t max_payload;
  uint8_t can_fit;
  uint8_t i;

  if ((pgns == nullptr) || (pgn_count == 0u)) {
    return 1u;
  }
  if ((uint16_t)(*offset + 3u + 4u) > max_len) {
    return 0u;
  }

  max_payload = (uint16_t)(max_len - *offset - 3u);
  can_fit = (uint8_t)(max_payload / 4u);
  if (can_fit > pgn_count) {
    can_fit = pgn_count;
  }
  if (can_fit == 0u) {
    return 0u;
  }

  msg[(*offset)++] = type;
  write_u16_le(&msg[*offset], (uint16_t)(can_fit * 4u));
  *offset = (uint16_t)(*offset + 2u);
  for (i = 0u; i < can_fit; i++) {
    write_u32_le(&msg[*offset], pgns[i]);
    *offset = (uint16_t)(*offset + 4u);
  }
  (*field_count)++;
  return 1u;
}

void update_product_info(DeviceInfo_t *dev, const uint8_t *payload, uint16_t len) {
  if ((dev == nullptr) || (payload == nullptr) || (len < 4u)) {
    return;
  }

  dev->product = read_u16_le(&payload[2]);
  /* Use progressive length checks so that devices sending a shorter-than-
   * standard PRODUCT_INFO payload still have their model_id captured.
   * Standard NMEA2000 PRODUCT_INFO is 134 bytes; the text fields begin at:
   *   model_id      offset  4, len 32 -> requires >= 36 bytes
   *   sw_version    offset 36, len 32 -> requires >= 68 bytes
   *   model_version offset 68, len 32 -> requires >= 100 bytes
   *   serial        offset 100, len 32 -> requires >= 132 bytes */
  if (len >= 36u)  { copy_n2k_text_field(dev->model_id,       &payload[4],   32u); }
  if (len >= 68u)  { copy_n2k_text_field(dev->sw_version,     &payload[36],  32u); }
  if (len >= 100u) { copy_n2k_text_field(dev->model_version,  &payload[68],  32u); }
  if (len >= 132u) { copy_n2k_text_field(dev->serial,         &payload[100], 32u); }
}

void update_config_info(DeviceInfo_t *dev, const uint8_t *payload, uint16_t len) {
  uint16_t idx = 0u;
  char temp[kTextMaxLen];
  if ((dev == nullptr) || (payload == nullptr)) {
    return;
  }

  if (!parse_var_string(payload, len, &idx, temp, sizeof(temp))) {
    return;
  }
  replace_text_field(dev->installation1, temp);

  if (!parse_var_string(payload, len, &idx, temp, sizeof(temp))) {
    return;
  }
  replace_text_field(dev->installation2, temp);

  if (parse_var_string(payload, len, &idx, temp, sizeof(temp))) {
    replace_text_field(dev->manufacturer_text, temp);
  }
}

void update_pgn_list(DeviceInfo_t *dev, const uint8_t *payload, uint16_t len) {
  uint8_t list_type;
  uint8_t count;

  if ((dev == nullptr) || (payload == nullptr) || (len < 1u)) {
    return;
  }

  list_type = payload[0];
  count = (uint8_t)((len - 1u) / 3u);
  if (count > kPgnMaxPerDirection) {
    count = kPgnMaxPerDirection;
  }

  if (list_type == 0u) {
    replace_pgn_list(dev->tx_pgn, dev->tx_pgn_count, payload, count);
  } else if (list_type == 1u) {
    replace_pgn_list(dev->rx_pgn, dev->rx_pgn_count, payload, count);
  }
}

/* -----------------------------------------------------------------------
 * Human-readable device table printed to UART when a snapshot completes.
 * Mirrors the PrintDeviceList() style of the Test_16 reference project.
 * ----------------------------------------------------------------------- */
static const char *resolve_manufacturer(uint16_t code) {
  switch (code) {
    case  69u: return "Actia";
    case 135u: return "Navico/Simrad/B&G";
    case 137u: return "Maretron";
    case 144u: return "Lowrance";
    case 147u: return "Furuno";
    case 148u: return "Trimble";
    case 154u: return "Garmin";
    case 163u: return "Yamaha";
    case 168u: return "SeaTalk/Raymarine";
    case 176u: return "Northstar";
    case 185u: return "Trimble";
    case 192u: return "ICOM";
    case 198u: return "Standard Horizon";
    case 211u: return "True Heading";
    case 215u: return "Cosworth";
    case 224u: return "Airmar";
    case 229u: return "Garmin";
    case 274u: return "Furuno";
    case 305u: return "B&G";
    case 306u: return "Silva";
    case 315u: return "Actisense";
    case 328u: return "Humminbird";
    case 344u: return "NovAtel";
    case 373u: return "Garmin";
    case 478u: return "Hemisphere GPS";
    case 579u: return "True Heading";
    case 824u: return "Rose Point";
    case 890u: return "Johnson Outdoors";
    case 1850u: return "Johnson Outdoors";
    case 2046u: return "Open Source/Experimental";
    default:   return nullptr;
  }
}

static const char *safe_str(const char *s) {
  return (s != nullptr && s[0] != '\0') ? s : "-";
}

static void print_pgn_row(const uint32_t *pgns, uint8_t count) {
  char buf[128];
  for (uint8_t i = 0u; i < count; i++) {
    uint8_t col = i % 8u;
    if (col == 0u) {
      (void)snprintf(buf, sizeof(buf), "    ");
      handler_uart_print(buf);
    }
    (void)snprintf(buf, sizeof(buf), "%lu%s",
      (unsigned long)pgns[i],
      (col == 7u || i == (count - 1u)) ? "\r\n" : ", ");
    handler_uart_print(buf);
  }
}

static void print_device_table(void) {
  char line[200];
  uint8_t count = count_seen_devices();
  uint32_t now  = HAL_GetTick();

  handler_uart_print("\r\n============================================================\r\n");
  (void)snprintf(line, sizeof(line),
    "[DEVICE TABLE] tick=%lums  total=%u devices\r\n",
    (unsigned long)now, (unsigned)count);
  handler_uart_print(line);
  handler_uart_print("------------------------------------------------------------\r\n");

  for (uint16_t i = 0u; i < kIdentitySlots; i++) {
    DeviceInfo_t *dev = s_devices[i];
    if ((dev == nullptr) || (dev->used == 0u)) {
      continue;
    }

    bool online = ((now - dev->last_seen_ms) < kDeviceOnlineMs);
    const char *mfr_name = resolve_manufacturer(dev->manufacturer);

    /* Line 1: source, model, manufacturer */
    if (mfr_name != nullptr) {
      (void)snprintf(line, sizeof(line),
        "Source %u  \"%s\"  Mfr: %s (%u)  UID: %lu  Product: %u\r\n",
        (unsigned)dev->source,
        safe_str(dev->model_id),
        mfr_name,
        (unsigned)dev->manufacturer,
        (unsigned long)dev->unique,
        (unsigned)dev->product);
    } else {
      (void)snprintf(line, sizeof(line),
        "Source %u  \"%s\"  Mfr: code=%u  UID: %lu  Product: %u\r\n",
        (unsigned)dev->source,
        safe_str(dev->model_id),
        (unsigned)dev->manufacturer,
        (unsigned long)dev->unique,
        (unsigned)dev->product);
    }
    handler_uart_print(line);

    /* Line 2: device class / function / instance */
    (void)snprintf(line, sizeof(line),
      "  Class: %u  Function: %u  Instance: %u/%u  IG: %u  %s\r\n",
      (unsigned)dev->device_class,
      (unsigned)dev->device_function,
      (unsigned)dev->device_instance,
      (unsigned)dev->system_instance,
      (unsigned)dev->industry_group,
      online ? "Online" : "Offline");
    handler_uart_print(line);

    /* Line 3: software/model version, serial */
    (void)snprintf(line, sizeof(line),
      "  SW: \"%s\"  Model ver: \"%s\"  Serial: \"%s\"\r\n",
      safe_str(dev->sw_version),
      safe_str(dev->model_version),
      safe_str(dev->serial));
    handler_uart_print(line);

    /* Line 4: manufacturer text + config strings */
    (void)snprintf(line, sizeof(line),
      "  Mfr text: \"%s\"  Install1: \"%s\"  Install2: \"%s\"\r\n",
      safe_str(dev->manufacturer_text),
      safe_str(dev->installation1),
      safe_str(dev->installation2));
    handler_uart_print(line);

    /* Line 5: flags and seen mask */
    (void)snprintf(line, sizeof(line),
      "  Flags: AC=%u PI=%u CI=%u TX=%u RX=%u  "
      "TX PGNs: %u  RX PGNs: %u  Last: %lums\r\n",
      (unsigned)((dev->seen_mask & kMaskAddressClaim) != 0u),
      (unsigned)((dev->seen_mask & kMaskProductInfo)  != 0u),
      (unsigned)((dev->seen_mask & kMaskConfigInfo)   != 0u),
      (unsigned)((dev->seen_mask & kMaskTxPgnList)    != 0u),
      (unsigned)((dev->seen_mask & kMaskRxPgnList)    != 0u),
      (unsigned)dev->tx_pgn_count,
      (unsigned)dev->rx_pgn_count,
      (unsigned long)dev->last_seen_ms);
    handler_uart_print(line);

    /* PGN lists */
    if ((dev->tx_pgn != nullptr) && (dev->tx_pgn_count > 0u)) {
      handler_uart_print("  TX PGNs:\r\n");
      print_pgn_row(dev->tx_pgn, dev->tx_pgn_count);
    }
    if ((dev->rx_pgn != nullptr) && (dev->rx_pgn_count > 0u)) {
      handler_uart_print("  RX PGNs:\r\n");
      print_pgn_row(dev->rx_pgn, dev->rx_pgn_count);
    }

    handler_uart_print("\r\n");
  }

  handler_uart_print("============================================================\r\n\r\n");
}

/* Queue a single text line as a STATUS (0x03) SPI packet so the ESP32 can
 * forward it via BLE to the Flutter app's DeviceListTextParser. */
static void queue_text_line_to_spi(const char *text) {
  uint8_t buf[SPI_PACKET_MAX_PAYLOAD_LEN];
  size_t len = strlen(text);
  if (len == 0u) {
    return;
  }
  if (len > (sizeof(buf) - 1u)) {
    len = sizeof(buf) - 1u;
  }
  (void)memcpy(buf, text, len);
  buf[len] = '\n';
  (void)N2K_RawBridge_QueueSpiPacket(SPI_PACKET_TYPE_STATUS, buf, (uint8_t)(len + 1u));
}

/* Sanitise a string value so it contains no '=' characters (which would
 * confuse the key=value text parser on the Flutter side). */
static void sanitise_field_value(char *dst, size_t dst_size, const char *src) {
  size_t i = 0u;
  if ((dst == nullptr) || (dst_size == 0u) || (src == nullptr) || (src[0] == '\0')) {
    if (dst != nullptr && dst_size > 0u) {
      dst[0] = '-'; dst[1] = '\0';
    }
    return;
  }
  for (; i < (dst_size - 1u) && src[i] != '\0'; i++) {
    dst[i] = (src[i] == '=') ? '_' : src[i];
  }
  dst[i] = '\0';
}

/* Send the full device list as text STATUS packets that the ESP32 forwards to
 * the Flutter app.  Format matches DeviceListTextParser expectations. */
static void send_device_list_text_to_ble(uint16_t request_id) {
  char line[220];
  char name_buf[64];
  char model_buf[64];
  char mfr_buf[40];
  uint8_t total = count_seen_devices();
  uint32_t now = HAL_GetTick();

  /* Header line */
  (void)snprintf(line, sizeof(line),
    "device_list snapshot id=%u complete=true expected=%u received=%u",
    (unsigned)request_id,
    (unsigned)total,
    (unsigned)total);
  queue_text_line_to_spi(line);

  /* One line per known device */
  for (uint16_t i = 0u; i < kIdentitySlots; i++) {
    DeviceInfo_t *dev = s_devices[i];
    if ((dev == nullptr) || (dev->used == 0u)) {
      continue;
    }
    bool online = ((now - dev->last_seen_ms) < kDeviceOnlineMs);
    bool is_gw  = (dev->source == kOwnN2kSource);

    sanitise_field_value(name_buf,  sizeof(name_buf),  safe_str(dev->model_id));
    sanitise_field_value(model_buf, sizeof(model_buf),
      (dev->model_version != nullptr && dev->model_version[0] != '\0')
        ? dev->model_version : safe_str(dev->model_id));

    const char *resolved = resolve_manufacturer(dev->manufacturer);
    sanitise_field_value(mfr_buf, sizeof(mfr_buf),
      (resolved != nullptr) ? resolved : "-");

    (void)snprintf(line, sizeof(line),
      "device src=%u name=%s model=%s mfg=%u manufacturer=%s"
      " online=%s hasaddressclaim=%s hasproductinfo=%s"
      " tx=%s rx=%s deviceclass=%u devicefunction=%u gateway=%s",
      (unsigned)dev->source,
      name_buf,
      model_buf,
      (unsigned)dev->manufacturer,
      mfr_buf,
      online    ? "true" : "false",
      (dev->seen_mask & kMaskAddressClaim) ? "true" : "false",
      (dev->seen_mask & kMaskProductInfo)  ? "true" : "false",
      (dev->tx_pgn_count > 0u) ? "true" : "false",
      (dev->rx_pgn_count > 0u) ? "true" : "false",
      (unsigned)dev->device_class,
      (unsigned)dev->device_function,
      is_gw ? "true" : "false");
    queue_text_line_to_spi(line);
  }
}

uint8_t emit_device_list_snapshot(uint16_t request_id) {
  uint8_t msg[SPI_PACKET_MAX_PAYLOAD_LEN];
  uint8_t total = count_seen_devices();
  uint8_t index = 0u;
  uint32_t snapshot_ms = HAL_GetTick();
  char line[200];

  write_common_header(msg, kDevListMsgBegin, request_id, snapshot_ms);
  msg[8] = kOwnN2kSource;
  msg[9] = total;
  if (queue_devlist_payload(msg, 10u) == 0u) {
    (void)snprintf(line, sizeof(line),
      "[SNAP] begin id=%u total=%u queue_failed\r\n",
      (unsigned)request_id,
      (unsigned)total);
    handler_uart_print(line);
    return 0u;
  }
  (void)snprintf(line, sizeof(line),
    "[SNAP] begin id=%u total=%u\r\n",
    (unsigned)request_id,
    (unsigned)total);
  handler_uart_print(line);

  for (uint16_t i = 0u; i < kIdentitySlots; i++) {
    DeviceInfo_t *dev = s_devices[i];
    uint16_t flags = 0u;
    uint16_t off = 39u;
    uint8_t field_count = 0u;
    uint8_t encode_ok = 1u;
    uint8_t tx_pgn_encoded = 0u;
    uint8_t rx_pgn_encoded = 0u;

    if ((dev == nullptr) || (dev->used == 0u)) {
      continue;
    }

    if ((uint32_t)(snapshot_ms - dev->last_seen_ms) < kDeviceOnlineMs) {
      flags |= kFlagOnline;
    }
    if ((dev->seen_mask & kMaskAddressClaim) != 0u) {
      flags |= kFlagHasAddressClaim;
    }
    if ((dev->seen_mask & kMaskProductInfo) != 0u) {
      flags |= kFlagHasProductInfo;
    }
    if ((dev->seen_mask & kMaskConfigInfo) != 0u) {
      flags |= kFlagHasConfigInfo;
    }
    write_common_header(msg, kDevListMsgDevice, request_id, snapshot_ms);
    msg[8] = index;
    msg[9] = total;
    msg[10] = dev->source;
    {
      uint8_t n;
      for (n = 0u; n < 8u; n++) {
        msg[13u + n] = (uint8_t)((dev->name >> (8u * n)) & 0xFFu);
      }
    }
    write_u32_le(&msg[21], dev->last_seen_ms);
    write_u32_le(&msg[25], dev->unique);
    write_u16_le(&msg[29], dev->manufacturer);
    write_u16_le(&msg[31], dev->product);
    msg[33] = dev->device_function;
    msg[34] = dev->device_class;
    msg[35] = dev->device_instance;
    msg[36] = dev->system_instance;
    msg[37] = dev->industry_group;

    encode_ok &= append_tlv_text(msg, &off, SPI_PACKET_MAX_PAYLOAD_LEN, kFieldModelId, dev->model_id, &field_count);
    encode_ok &= append_tlv_text(msg, &off, SPI_PACKET_MAX_PAYLOAD_LEN, kFieldSoftwareVersion, dev->sw_version, &field_count);
    encode_ok &= append_tlv_text(msg, &off, SPI_PACKET_MAX_PAYLOAD_LEN, kFieldModelVersion, dev->model_version, &field_count);
    encode_ok &= append_tlv_text(msg, &off, SPI_PACKET_MAX_PAYLOAD_LEN, kFieldSerialCode, dev->serial, &field_count);
    encode_ok &= append_tlv_text(msg, &off, SPI_PACKET_MAX_PAYLOAD_LEN, kFieldManufacturerText, dev->manufacturer_text, &field_count);
    encode_ok &= append_tlv_text(msg, &off, SPI_PACKET_MAX_PAYLOAD_LEN, kFieldInstallation1, dev->installation1, &field_count);
    encode_ok &= append_tlv_text(msg, &off, SPI_PACKET_MAX_PAYLOAD_LEN, kFieldInstallation2, dev->installation2, &field_count);
    tx_pgn_encoded = append_tlv_pgn_list(msg, &off, SPI_PACKET_MAX_PAYLOAD_LEN, kFieldTxPgnList, dev->tx_pgn, dev->tx_pgn_count, &field_count);
    rx_pgn_encoded = append_tlv_pgn_list(msg, &off, SPI_PACKET_MAX_PAYLOAD_LEN, kFieldRxPgnList, dev->rx_pgn, dev->rx_pgn_count, &field_count);

    if (encode_ok == 0u) {
      (void)snprintf(line, sizeof(line),
        "[SNAP] dev src=%u encode_failed tx=%u rx=%u off=%u\r\n",
        (unsigned)dev->source,
        (unsigned)dev->tx_pgn_count,
        (unsigned)dev->rx_pgn_count,
        (unsigned)off);
      handler_uart_print(line);
      return 0u;
    }

    if ((dev->tx_pgn_count > 0u) && (tx_pgn_encoded != 0u)) {
      flags |= kFlagHasTxPgnList;
    }
    if ((dev->rx_pgn_count > 0u) && (rx_pgn_encoded != 0u)) {
      flags |= kFlagHasRxPgnList;
    }

    write_u16_le(&msg[11], flags);

    msg[38] = field_count;
    if (queue_devlist_payload(msg, (uint8_t)off) == 0u) {
      (void)snprintf(line, sizeof(line),
        "[SNAP] dev src=%u len=%u fields=%u queue_failed tx=%u/%u rx=%u/%u\r\n",
        (unsigned)dev->source,
        (unsigned)off,
        (unsigned)field_count,
        (unsigned)tx_pgn_encoded,
        (unsigned)dev->tx_pgn_count,
        (unsigned)rx_pgn_encoded,
        (unsigned)dev->rx_pgn_count);
      handler_uart_print(line);
      return 0u;
    }
    (void)snprintf(line, sizeof(line),
      "[SNAP] dev src=%u len=%u fields=%u tx=%u/%u rx=%u/%u flags=0x%04X\r\n",
      (unsigned)dev->source,
      (unsigned)off,
      (unsigned)field_count,
      (unsigned)tx_pgn_encoded,
      (unsigned)dev->tx_pgn_count,
      (unsigned)rx_pgn_encoded,
      (unsigned)dev->rx_pgn_count,
      (unsigned)flags);
    handler_uart_print(line);
    index++;
  }

  write_common_header(msg, kDevListMsgEnd, request_id, snapshot_ms);
  msg[8] = total;
  if (queue_devlist_payload(msg, 9u) == 0u) {
    (void)snprintf(line, sizeof(line),
      "[SNAP] end id=%u total=%u queue_failed\r\n",
      (unsigned)request_id,
      (unsigned)total);
    handler_uart_print(line);
    return 0u;
  }

  (void)snprintf(line, sizeof(line),
    "[SNAP] end id=%u total=%u encoded=%u\r\n",
    (unsigned)request_id,
    (unsigned)total,
    (unsigned)index);
  handler_uart_print(line);

  print_device_table();
  send_device_list_text_to_ble(request_id);

  return 1u;
}

void mark_seen(uint8_t source, uint8_t bit) {
  DeviceInfo_t *dev = find_or_alloc_device(source);
  if (dev == nullptr) {
    return;
  }
  dev->last_seen_ms = HAL_GetTick();
  dev->seen_mask |= bit;
}

uint8_t send_follow_up_request(uint8_t dst, uint32_t requested_pgn, const char *label) {
  char line[160];
  uint8_t sent = N2K_RawBridge_SendIsoRequest(kIsoRequestSrc, dst, requested_pgn);

  (void)snprintf(line, sizeof(line),
    "[REQ:FOLLOWUP] src=%u req=%lu %s result=%s\r\n",
    (unsigned)dst,
    (unsigned long)requested_pgn,
    (label != nullptr) ? label : "-",
    (sent != 0u) ? "ok" : "fail");
  handler_uart_print(line);
  return sent;
}

void request_follow_up_metadata(DeviceInfo_t *dev) {
  if ((dev == nullptr) || (dev->source == kOwnN2kSource)) {
    return;
  }

  if (((dev->request_mask & kRequestProductInfo) == 0u) &&
      send_follow_up_request(dev->source, kPgnProductInfo, "product") != 0u) {
    dev->request_mask |= kRequestProductInfo;
  }

  if (((dev->request_mask & kRequestConfigInfo) == 0u) &&
      send_follow_up_request(dev->source, kPgnConfigInfo, "config") != 0u) {
    dev->request_mask |= kRequestConfigInfo;
  }

  if (((dev->request_mask & kRequestPgnList) == 0u) &&
      send_follow_up_request(dev->source, kPgnPgnList, "pgn-list") != 0u) {
    dev->request_mask |= kRequestPgnList;
  }
}

void schedule_snapshot_retry(uint16_t request_id) {
  s_snapshot_retry_pending = 1u;
  s_snapshot_retry_request_id = request_id;
  s_snapshot_retry_attempt = 0u;
  s_snapshot_retry_at_ms = HAL_GetTick();
}

void process_snapshot_retry(void) {
  char line[120];
  uint32_t now;
  uint8_t ok;

  if (s_snapshot_retry_pending == 0u) {
    return;
  }

  now = HAL_GetTick();
  if ((int32_t)(now - s_snapshot_retry_at_ms) < 0) {
    return;
  }

  ok = emit_device_list_snapshot(s_snapshot_retry_request_id);
  if (ok != 0u) {
    if (s_snapshot_retry_attempt > 0u) {
      (void)snprintf(line, sizeof(line),
        "[SPI] snapshot retry ok id=%u retries=%u qovf=%lu\r\n",
        (unsigned)s_snapshot_retry_request_id,
        (unsigned)s_snapshot_retry_attempt,
        (unsigned long)s_devlist_queue_overflow_count);
      handler_uart_print(line);
    }
    s_snapshot_retry_pending = 0u;
    return;
  }

  s_snapshot_retry_attempt++;
  s_devlist_retry_count++;
  if (s_snapshot_retry_attempt > kSnapshotMaxRetries) {
    s_devlist_retry_giveup_count++;
    (void)snprintf(line, sizeof(line),
      "[SPI] snapshot retry giveup id=%u tries=%u qovf=%lu\r\n",
      (unsigned)s_snapshot_retry_request_id,
      (unsigned)s_snapshot_retry_attempt,
      (unsigned long)s_devlist_queue_overflow_count);
    handler_uart_print(line);
    s_snapshot_retry_pending = 0u;
    return;
  }

  s_snapshot_retry_at_ms = now + kSnapshotRetryBackoffMs;
}

/* Soft reset for a new refresh cycle: keep device identity and cached text
   data so previously-known devices still appear immediately in the new
   snapshot.  Only reset the tracking bits so fresh PGN-list requests are
   re-sent for all known devices.  This fixes the "deleted / re-connected
   device never shows up again" symptom. */
static void soft_reset_for_refresh(void) {
  for (uint16_t i = 0u; i < kIdentitySlots; i++) {
    if (s_devices[i] != nullptr) {
      s_devices[i]->request_mask = 0u;               /* allow re-requesting metadata */
      s_devices[i]->seen_mask   &= kMaskAddressClaim; /* keep AC bit, clear PGN/prod/cfg */
    }
  }
}

void start_refresh_request(uint16_t req_id, uint32_t requested_pgn, uint8_t dst, const char *origin) {
  char line[180];
  uint8_t sent;
  uint8_t had_devices = count_seen_devices();

  sent = N2K_RawBridge_SendIsoRequest(kIsoRequestSrc, dst, requested_pgn);
  if (sent != 0u) {
    /* Soft reset: keep old device data for faster re-display; only clear
       tracking state so metadata is re-fetched for all devices. */
    soft_reset_for_refresh();
    (void)snprintf(line, sizeof(line),
      "[SRESET] keeping %u known devices, resetting tracking\r\n",
      (unsigned)had_devices);
    handler_uart_print(line);
    s_refresh_pending = 1u;
    s_active_request_id = req_id;
    s_refresh_started_ms = HAL_GetTick();
    s_refresh_last_activity_ms = s_refresh_started_ms;
    s_snapshot_retry_pending = 0u;
    (void)snprintf(line, sizeof(line),
      "[REQ:%s] id=%u tick=%lu src=%u dst=%u pgn=%lu req=%lu data=%02X %02X %02X result=ok\r\n",
      origin,
      (unsigned)req_id,
      (unsigned long)HAL_GetTick(),
      (unsigned)kIsoRequestSrc,
      (unsigned)dst,
      (unsigned long)kPgnIsoRequest,
      (unsigned long)requested_pgn,
      (unsigned)(requested_pgn & 0xFFu),
      (unsigned)((requested_pgn >> 8u) & 0xFFu),
      (unsigned)((requested_pgn >> 16u) & 0xFFu));
    handler_uart_print(line);
  } else {
    (void)snprintf(line, sizeof(line),
      "[REQ:%s] id=%u pgn=%lu req=%lu result=fail\r\n",
      origin,
      (unsigned)req_id,
      (unsigned long)kPgnIsoRequest,
      (unsigned long)requested_pgn);
    handler_uart_print(line);
  }
}
}

void DeviceListHandler_Init(void) {
  reset_refresh_tracking();
  N2K_RawBridge_ResetSeenSources();
  s_refresh_pending = 0u;
  s_request_id = 1u;
  s_active_request_id = 0u;
  s_refresh_started_ms = 0u;
  s_refresh_last_activity_ms = 0u;
  s_snapshot_retry_pending = 0u;
  s_snapshot_retry_request_id = 0u;
  s_snapshot_retry_attempt = 0u;
  s_snapshot_retry_at_ms = 0u;
  s_devlist_queue_overflow_count = 0u;
  s_devlist_retry_count = 0u;
  s_devlist_retry_giveup_count = 0u;
  s_devlist_alloc_fail_count = 0u;

  /* Send an ISO Request for Address Claim on boot so the app receives
     device identity frames as soon as the N2K bus is ready.
     Raw responses are forwarded via n2k_raw_bridge.c -> SPI -> ESP32 -> BLE.
     The Flutter app's N2kDeviceTracker builds the device list from those frames.
     s_refresh_pending must be 1 so that OnRxEvent processes the ADDR_CLAIM
     responses and sends follow-up ISO requests for PRODUCT_INFO etc. */
  N2K_RawBridge_SendIsoRequest(kIsoRequestSrc, kIsoDstGlobal, kPgnAddressClaim);
  s_refresh_pending = 1u;
  s_refresh_started_ms = HAL_GetTick();
  s_refresh_last_activity_ms = s_refresh_started_ms;
  handler_uart_print("[INIT] ISO Request broadcast sent (addr claim)\r\n");
  handler_uart_print("[CMD] Type 'r' to re-request device list\r\n");
}

void DeviceListHandler_PollUart(void) {
  uint8_t rx = 0u;
  HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, &rx, 1u, 0u);
  char line[160];

  if (status != HAL_OK) {
    return;
  }

  if ((rx == 'r') || (rx == 'R')) {
    uint16_t req_id = s_request_id;
    (void)line;
    start_refresh_request(req_id, kPgnAddressClaim, kIsoDstGlobal, "UART");

    s_request_id++;
    if (s_request_id == 0u) {
      s_request_id = 1u;
    }
  }
}

void DeviceListHandler_OnSpiRequest(const uint8_t *payload, uint8_t payload_len) {
  uint16_t req_id = s_request_id;
  uint32_t requested_pgn = kPgnAddressClaim;
  uint8_t dst = kIsoDstGlobal;

  if ((payload != nullptr) && (payload_len >= 8u)) {
    req_id = read_u16_le(&payload[1]);
    dst = payload[4];
    requested_pgn = ((uint32_t)payload[5]) |
                    ((uint32_t)payload[6] << 8u) |
                    ((uint32_t)payload[7] << 16u);
    if (requested_pgn == 0u) {
      requested_pgn = kPgnAddressClaim;
    }
    if (dst == 0u) {
      dst = kIsoDstGlobal;
    }
  }

  start_refresh_request(req_id, requested_pgn, dst, "SPI");

  if (req_id >= s_request_id) {
    s_request_id = (uint16_t)(req_id + 1u);
    if (s_request_id == 0u) {
      s_request_id = 1u;
    }
  }
}

void DeviceListHandler_OnRxEvent(const N2K_RawBridgeRxEvent_t *event) {
  DeviceInfo_t *dev;
  uint64_t name;

  if ((event == nullptr) || (s_refresh_pending == 0u)) {
    return;
  }

  if (event->pgn != kPgnAddressClaim) {
    return;
  }
  if (event->dlc < 8u) {
    return;
  }

  dev = find_or_alloc_device(event->src);
  if (dev == nullptr) {
    return;
  }
  name = read_u64_le(event->data);
  dev->name = name;
  dev->unique = (uint32_t)(name & 0x1FFFFFu);
  dev->manufacturer = (uint16_t)((name >> 21u) & 0x7FFu);
  dev->device_instance = (uint8_t)((name >> 32u) & 0xFFu);
  dev->device_function = (uint8_t)((name >> 40u) & 0xFFu);
  dev->device_class = (uint8_t)(((name >> 48u) & 0xFFu) >> 1u);
  dev->system_instance = (uint8_t)((name >> 56u) & 0x0Fu);
  dev->industry_group = (uint8_t)((name >> 60u) & 0x07u);

  mark_seen(event->src, kMaskAddressClaim);

  {
    char line[160];
    (void)snprintf(line, sizeof(line),
      "[AC] src=%u mfr=%u uid=%lu cls=%u fn=%u ig=%u inst=%u\r\n",
      (unsigned)event->src,
      (unsigned)dev->manufacturer,
      (unsigned long)dev->unique,
      (unsigned)dev->device_class,
      (unsigned)dev->device_function,
      (unsigned)dev->industry_group,
      (unsigned)dev->device_instance);
    handler_uart_print(line);
  }

  request_follow_up_metadata(dev);
  s_refresh_last_activity_ms = HAL_GetTick();
}

void DeviceListHandler_OnAssembledEvent(const N2K_RawBridgeAssembledEvent_t *event) {
  DeviceInfo_t *dev;

  if ((event == nullptr) || (s_refresh_pending == 0u)) {
    return;
  }

  if ((event->pgn != kPgnProductInfo) &&
      (event->pgn != kPgnConfigInfo) &&
      (event->pgn != kPgnPgnList)) {
    return;
  }

  dev = find_or_alloc_device(event->src);
  if (dev == nullptr) {
    return;
  }
  if (event->pgn == kPgnProductInfo) {
    update_product_info(dev, event->data, event->len);
    mark_seen(event->src, kMaskProductInfo);
    {
      char line[160];
      (void)snprintf(line, sizeof(line),
        "[META] src=%u product_info len=%u model='%s' sw='%s'\r\n",
        (unsigned)event->src,
        (unsigned)event->len,
        (dev->model_id != nullptr) ? dev->model_id : "",
        (dev->sw_version != nullptr) ? dev->sw_version : "");
      handler_uart_print(line);
    }
  } else if (event->pgn == kPgnConfigInfo) {
    update_config_info(dev, event->data, event->len);
    mark_seen(event->src, kMaskConfigInfo);
    {
      char line[160];
      (void)snprintf(line, sizeof(line),
        "[META] src=%u config_info len=%u inst1='%s' inst2='%s'\r\n",
        (unsigned)event->src,
        (unsigned)event->len,
        (dev->installation1 != nullptr) ? dev->installation1 : "",
        (dev->installation2 != nullptr) ? dev->installation2 : "");
      handler_uart_print(line);
    }
  } else {
    update_pgn_list(dev, event->data, event->len);
    if ((event->len > 0u) && (event->data[0] == 0u)) {
      mark_seen(event->src, kMaskTxPgnList);
      {
        char line[120];
        (void)snprintf(line, sizeof(line),
          "[META] src=%u tx_pgn_list count=%u\r\n",
          (unsigned)event->src,
          (unsigned)dev->tx_pgn_count);
        handler_uart_print(line);
      }
    } else if ((event->len > 0u) && (event->data[0] == 1u)) {
      mark_seen(event->src, kMaskRxPgnList);
      {
        char line[120];
        (void)snprintf(line, sizeof(line),
          "[META] src=%u rx_pgn_list count=%u\r\n",
          (unsigned)event->src,
          (unsigned)dev->rx_pgn_count);
        handler_uart_print(line);
      }
    }
  }

  s_refresh_last_activity_ms = HAL_GetTick();
}

void DeviceListHandler_PollState(void) {
  uint8_t src;

  /* Drain the new-source queue.  When a device broadcasts data PGNs
   * (wind, GPS, depth, etc.) without responding to the boot-time
   * broadcast ISO Request for ADDR_CLAIM, we send a unicast ISO Request
   * directly to it.  Most N2K devices respond to unicast requests even
   * when they silently ignore the broadcast. */
  while (N2K_RawBridge_PopNewSource(&src) != 0u) {
    DeviceInfo_t *dev;
    if ((src == kOwnN2kSource)) {
      continue;
    }
    dev = find_or_alloc_device(src);
    if ((dev != nullptr) &&
        ((dev->seen_mask & kMaskAddressClaim) == 0u) &&
        ((dev->request_mask & 0x80u) == 0u)) {
      dev->request_mask |= 0x80u;  /* bit 7: unicast addr-claim solicited */
      send_follow_up_request(src, kPgnAddressClaim, "sniff");
    }
  }

  /* ---- Scan-completion check -------------------------------------------- *
   * Fire the snapshot when either:
   *   a) kRefreshQuietWindowMs has passed with no new AddressClaim activity, OR
   *   b) kRefreshTimeoutMs has elapsed since the scan started (hard deadline).
   * Both conditions require s_refresh_pending to be set. */
  if (s_refresh_pending != 0u) {
    uint32_t now = HAL_GetTick();
    uint32_t quiet_ms  = (uint32_t)(now - s_refresh_last_activity_ms);
    uint32_t total_ms  = (uint32_t)(now - s_refresh_started_ms);

    if ((quiet_ms >= kRefreshQuietWindowMs) ||
        (total_ms >= kRefreshTimeoutMs)) {
      uint8_t ok;
      char line[120];
      (void)snprintf(line, sizeof(line),
        "[SCAN] complete quiet=%lums total=%lums devices=%u\r\n",
        (unsigned long)quiet_ms,
        (unsigned long)total_ms,
        (unsigned)count_seen_devices());
      handler_uart_print(line);

      ok = emit_device_list_snapshot(s_active_request_id);
      if (ok == 0u) {
        schedule_snapshot_retry(s_active_request_id);
      }
      s_refresh_pending = 0u;  /* close this scan cycle */
    }
  }

  /* ---- Periodic auto-rescan -------------------------------------------- *
   * If no scan is active and ≥30 s have passed since the last one, start a
   * new broadcast scan.  This picks up devices that were slow to boot or
   * that didn't respond to the initial request. */
  if (s_refresh_pending == 0u) {
    static uint32_t s_last_rescan_ms = 0u;
    uint32_t now2 = HAL_GetTick();
    if (s_last_rescan_ms == 0u) {
      s_last_rescan_ms = now2;  /* skip first 30 s after boot */
    } else if ((uint32_t)(now2 - s_last_rescan_ms) >= 30000u) {
      s_last_rescan_ms = now2;
      start_refresh_request(s_request_id, kPgnAddressClaim, kIsoDstGlobal, "AUTO");
      s_request_id++;
      if (s_request_id == 0u) {
        s_request_id = 1u;
      }
    }
  }

  /* Handle any pending SPI-queue retry (independent of active scan). */
  process_snapshot_retry();
}
