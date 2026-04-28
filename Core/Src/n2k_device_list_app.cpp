#include "n2k_device_list_app.hpp"

#include "n2k_raw_bridge.h"
#include "spi_packet.h"
#include "N2kMessages.h"
#include "stm32f1xx_hal.h"

#include <string.h>
#include <stdint.h>

/* ── millis() / delay() required by the NMEA2000 library ───────────────── */
/* (Not provided by NMEA2000_STM32.cpp because we use our own CAN adapter.) */
extern "C" uint32_t millis() { return HAL_GetTick(); }
extern "C" void     delay(uint32_t ms) { HAL_Delay(ms); }

/* ── Helpers ──────────────────────────────────────────────────────────────*/
static const char *SafeStr(const char *s) { return (s != nullptr) ? s : ""; }

static bool IsRefreshPgn(uint32_t pgn) {
    return (pgn == 60928UL  ||   /* AddressClaim    */
            pgn == 126996UL ||   /* ProductInfo     */
            pgn == 126998UL ||   /* ConfigInfo      */
            pgn == 126464UL);    /* PGN List        */
}

/* ── N2kBridgeNode ──────────────────────────────────────────────────────── */

bool N2kBridgeNode::CANOpen() {
    return true; /* CAN already started by N2K_RawBridge_Init() */
}

bool N2kBridgeNode::CANSendFrame(unsigned long id, unsigned char len,
                                  const unsigned char *buf, bool wait_sent) {
    (void)wait_sent;
    return N2K_RawBridge_TransmitRawFrame(id, len, buf) != 0u;
}

bool N2kBridgeNode::CANGetFrame(unsigned long &id, unsigned char &len,
                                 unsigned char *buf) {
    return N2K_RawBridge_N2kFeedPop(&id, &len, buf) != 0u;
}

/* ── N2kDeviceListApp ───────────────────────────────────────────────────── */

N2kDeviceListApp *N2kDeviceListApp::s_instance_ = nullptr;

N2kDeviceListApp::N2kDeviceListApp()
    : n2k_(), deviceList_(&n2k_) {}

void N2kDeviceListApp::Begin() {
    s_instance_ = this;

    n2k_.SetProductInformation(
        "00000015",
        100,
        "N2K SPI Bridge",
        "1.0.0.0 (2026-04-27)",
        "1.0.0.0 (2026-04-27)");

    n2k_.SetDeviceInformation(
        15,    /* unique instance number */
        130,   /* device function: Display */
        120,   /* device class: Navigation */
        2046); /* manufacturer code: Open Source */

    n2k_.SetMode(tNMEA2000::N2km_ListenAndNode, 15U);
    n2k_.EnableForward(false);
    n2k_.SetMsgHandler(StaticMessageHandler);
    n2k_.SetMaxCANSendFrames(8U);
    n2k_.Open();
}

void N2kDeviceListApp::Poll() {
    n2k_.ParseMessages();

    if (!refreshPending_) {
        return;
    }

    if (deviceList_.ReadResetIsListUpdated()) {
        lastActivityMs_ = HAL_GetTick();
    }

    uint32_t now = HAL_GetTick();
    bool quietDone = ((int32_t)(now - lastActivityMs_)) >= (int32_t)kQuietWindowMs;
    bool maxDone   = ((int32_t)(now - refreshStartedMs_)) >= (int32_t)kMaxWaitMs;

    if (quietDone || maxDone) {
        refreshPending_ = false;
        SendDeviceList();
    }
}

void N2kDeviceListApp::RequestDeviceList() {
    if (refreshPending_) {
        N2K_RawBridge_QueueSpiPacket(SPI_PACKET_TYPE_SCAN_BUSY, nullptr, 0U);
        return;
    }

    if (!n2k_.IsOpen()) {
        N2K_RawBridge_QueueSpiPacket(SPI_PACKET_TYPE_SCAN_NO_BUS, nullptr, 0U);
        return;
    }

    /* Send a single broadcast ISO request for AddressClaim.
     * tN2kDeviceList will automatically issue follow-up requests for
     * ProductInfo, ConfigInfo, and PGN List when it sees new devices.      */
    tN2kMsg requestMsg;
    SetN2kPGNISORequest(requestMsg, 0xffU, N2kPGNIsoAddressClaim);
    if (!n2k_.SendMsg(requestMsg, -1)) {
        return;
    }

    refreshPending_   = true;
    refreshStartedMs_ = HAL_GetTick();
    lastActivityMs_   = HAL_GetTick();

    /* Notify ESP32/Flutter that the scan has started. */
    SpiPld_ScanAck_t ack;
    ack.requested_pgn = (uint32_t)N2kPGNIsoAddressClaim;
    ack.own_source    = n2k_.GetN2kSource();
    N2K_RawBridge_QueueSpiPacket(SPI_PACKET_TYPE_SCAN_ACK,
                                  reinterpret_cast<const uint8_t *>(&ack),
                                  (uint8_t)sizeof(ack));
}

void N2kDeviceListApp::SendDeviceList() {
    /* Max string length per field (+ NUL = 21 bytes each × 7 = 147 bytes).
     * Fixed header = 20 bytes.  Total ≤ 167 bytes, well under the 250-byte
     * SPI payload cap and the 180-byte BLE MTU.                            */
    static const size_t kStrMax = 20U;

    uint8_t deviceCount = 0U;

    for (uint16_t src = 0U; src < N2kMaxBusDevices; src++) {
        if (src == (uint16_t)n2k_.GetN2kSource()) {
            continue;
        }

        const tNMEA2000::tDevice *dev =
            deviceList_.FindDeviceBySource((uint8_t)src);
        if (dev == nullptr) {
            continue;
        }

        deviceCount++;

        /* Build: fixed header immediately followed by 7 null-terminated strings */
        uint8_t  buf[sizeof(SpiPld_DeviceEntry_t) + 7U * (kStrMax + 1U)];
        uint16_t pos = 0U;

        auto *hdr = reinterpret_cast<SpiPld_DeviceEntry_t *>(buf);
        hdr->source            = (uint8_t)src;
        hdr->online            = 1U;
        hdr->manufacturer_code = dev->GetManufacturerCode();
        hdr->unique_number     = (uint32_t)dev->GetUniqueNumber();
        hdr->device_function   = dev->GetDeviceFunction();
        hdr->device_class      = dev->GetDeviceClass();
        hdr->device_instance   = (uint8_t)dev->GetDeviceInstance();
        hdr->system_instance   = (uint8_t)dev->GetSystemInstance();
        hdr->industry_group    = dev->GetIndustryGroup();
        hdr->reserved          = 0U;
        hdr->product_code      = dev->GetProductCode();
        hdr->last_msg_tick_ms  = HAL_GetTick();
        pos = (uint16_t)sizeof(SpiPld_DeviceEntry_t);

        const char *strings[7] = {
            SafeStr(dev->GetModelID()),
            SafeStr(dev->GetSwCode()),
            SafeStr(dev->GetModelVersion()),
            SafeStr(dev->GetModelSerialCode()),
            SafeStr(dev->GetManufacturerInformation()),
            SafeStr(dev->GetInstallationDescription1()),
            SafeStr(dev->GetInstallationDescription2()),
        };

        for (uint8_t si = 0U; si < 7U; si++) {
            size_t slen = strnlen(strings[si], kStrMax);
            if ((size_t)(pos + slen + 1U) > sizeof(buf)) {
                slen = 0U; /* safety clamp */
            }
            memcpy(&buf[pos], strings[si], slen);
            pos += (uint16_t)slen;
            buf[pos] = '\0';
            pos++;
        }

        N2K_RawBridge_QueueSpiPacket(SPI_PACKET_TYPE_DEVICE_ENTRY,
                                      buf, (uint8_t)pos);
    }

    /* Signal end of list. */
    SpiPld_ListDone_t done;
    done.device_count = deviceCount;
    N2K_RawBridge_QueueSpiPacket(SPI_PACKET_TYPE_LIST_DONE,
                                  reinterpret_cast<const uint8_t *>(&done),
                                  (uint8_t)sizeof(done));
}

void N2kDeviceListApp::HandleMessage(const tN2kMsg &msg) {
    /* Reset the quiet-window timer whenever a device-list related message
     * arrives from any remote node while a refresh is in progress.         */
    if (refreshPending_ &&
        msg.Source != n2k_.GetN2kSource() &&
        IsRefreshPgn(msg.PGN)) {
        lastActivityMs_ = HAL_GetTick();
    }
}

void N2kDeviceListApp::StaticMessageHandler(const tN2kMsg &msg) {
    if (s_instance_ != nullptr) {
        s_instance_->HandleMessage(msg);
    }
}
