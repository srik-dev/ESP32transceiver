idf_component_register(
    SRCS "beacon_tx.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_event nvs_flash console esp_timer esp_netif)
