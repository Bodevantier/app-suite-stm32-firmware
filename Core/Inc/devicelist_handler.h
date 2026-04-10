#ifndef DEVICE_LIST_HANDLER_H
#define DEVICE_LIST_HANDLER_H

#include "n2k_raw_bridge.h"

#ifdef __cplusplus
extern "C" {
#endif

void DeviceListHandler_Init(void);
void DeviceListHandler_PollUart(void);
void DeviceListHandler_OnSpiRequest(const uint8_t *payload, uint8_t payload_len);
void DeviceListHandler_OnRxEvent(const N2K_RawBridgeRxEvent_t *event);
void DeviceListHandler_OnAssembledEvent(const N2K_RawBridgeAssembledEvent_t *event);
void DeviceListHandler_PollState(void);

#ifdef __cplusplus
}
#endif

#endif
