#include "wifi_ap.h"
#include "config.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"

static const char *TAG = "WiFi-AP";
static bool wifi_ap_started = false;
static int connected_clients = 0;

void wifi_ap_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        connected_clients++;
        ESP_LOGI(TAG, "Client connected. Total clients: %d", connected_clients);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        connected_clients--;
        ESP_LOGI(TAG, "Client disconnected. Total clients: %d", connected_clients);
    } else if (event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "Access Point started successfully!");
        wifi_ap_started = true;
    } else if (event_id == WIFI_EVENT_AP_STOP) {
        ESP_LOGI(TAG, "Access Point stopped");
        wifi_ap_started = false;
    }
}

esp_err_t wifi_init_ap(void) {
    ESP_LOGI(TAG, "Initializing WiFi Access Point...");
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);        // IP 
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);        // Gateway
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0); // Netmask
    
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_ap_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASS,
            .max_connection = WIFI_AP_MAX_CONNECTIONS,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = true,
            },
        },
    };

    if (strlen(WIFI_AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE); 

    ESP_LOGI(TAG, "WiFi Access Point initialized");
    ESP_LOGI(TAG, "SSID: %s", WIFI_AP_SSID);
    ESP_LOGI(TAG, "Password: %s", WIFI_AP_PASS);
    ESP_LOGI(TAG, "IP Address: 192.168.4.1");
    ESP_LOGI(TAG, "Connect your device and use IP: 192.168.4.1:%d", TCP_PORT);

    return ESP_OK;
}

bool wifi_is_ap_started(void) {
    return wifi_ap_started;
}

int wifi_get_connected_clients(void) {
    return connected_clients;
}