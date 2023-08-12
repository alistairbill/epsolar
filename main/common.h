#ifndef __COMMON_H__
#define __COMMON_H__

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <nvs.h>

#define WIFI_CONNECTED_BIT BIT0
#define READ_COMPLETED_BIT BIT1
#define PUBLISH_COMPLETED_BIT BIT2

extern volatile EventGroupHandle_t services_event_group;

#endif