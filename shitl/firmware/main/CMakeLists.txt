idf_component_register(
    SRCS "shitl_tx.c" "net.c" "ble_tx.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_event nvs_flash console esp_timer esp_netif bt
             esp_http_server lwip app_update esp_app_format)
