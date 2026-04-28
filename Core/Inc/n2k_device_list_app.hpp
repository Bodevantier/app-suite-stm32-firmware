#pragma once

#include "NMEA2000.h"
#include "N2kDeviceList.h"

#ifdef __cplusplus

/* ── N2kBridgeNode ─────────────────────────────────────────────────────────
 * Custom tNMEA2000 adapter.  CAN TX is routed through the existing
 * N2K_RawBridge_TransmitRawFrame() so no STM32_CAN library is needed and
 * the existing HAL ISR / ring-buffer stays in control of the CAN peripheral.
 * CAN RX is supplied from the N2K feed ring buffer that the ISR fills.    */
class N2kBridgeNode : public tNMEA2000 {
protected:
    bool CANSendFrame(unsigned long id, unsigned char len,
                      const unsigned char *buf, bool wait_sent = true) override;
    bool CANOpen() override;
    bool CANGetFrame(unsigned long &id, unsigned char &len,
                     unsigned char *buf) override;
};

/* ── N2kDeviceListApp ──────────────────────────────────────────────────────
 * Mirrors Test-16's tN2kDeviceListApp but sends structured SPI packets
 * instead of UART frames.
 *
 * Scan timing (same as Test-16):
 *   - 500 ms quiet window: no device-list PGN activity → list complete
 *   - 2000 ms hard cap: send list regardless
 *
 * SPI packets sent:
 *   SCAN_ACK     – scan started (own source address + requested PGN)
 *   SCAN_BUSY    – scan already running
 *   SCAN_NO_BUS  – NMEA2000 bus not open
 *   DEVICE_ENTRY – one device (fixed header + 7 null-terminated strings)
 *   LIST_DONE    – all entries sent                                        */
class N2kDeviceListApp {
public:
    N2kDeviceListApp();

    /* Call once after peripherals are initialised.                          */
    void Begin();

    /* Call every main-loop iteration.                                       */
    void Poll();

    /* Call when a DEVICE_LIST_REQUEST SPI packet is received from the ESP32.*/
    void RequestDeviceList();

private:
    static constexpr uint32_t kQuietWindowMs = 500U;
    static constexpr uint32_t kMaxWaitMs     = 2000U;

    void SendDeviceList();
    void HandleMessage(const tN2kMsg &msg);
    static void StaticMessageHandler(const tN2kMsg &msg);

    static N2kDeviceListApp *s_instance_;

    N2kBridgeNode  n2k_;
    tN2kDeviceList deviceList_;

    bool     refreshPending_   = false;
    uint32_t refreshStartedMs_ = 0U;
    uint32_t lastActivityMs_   = 0U;
    uint8_t  txSeq_            = 0U;
};

#endif /* __cplusplus */
