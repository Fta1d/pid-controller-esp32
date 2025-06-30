#ifndef WIFI_AP_H
#define WIFI_AP_H

#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"

esp_err_t wifi_init_ap(void);

void wifi_ap_event_handler(void* arg, esp_event_base_t event_base, 
                          int32_t event_id, void* event_data);

bool wifi_is_ap_started(void);
int wifi_get_connected_clients(void);

#endif // WIFI_AP_H