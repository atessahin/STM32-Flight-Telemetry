idf_component_register(SRCS "udp_server.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_wifi esp_netif nvs_flash lwip telemetry_core)