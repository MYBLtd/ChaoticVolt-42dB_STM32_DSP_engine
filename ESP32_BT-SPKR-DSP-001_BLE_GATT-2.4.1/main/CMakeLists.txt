idf_component_register(SRCS "main.c"
                            "ble_gatt_dsp.c"
                            "nvs_settings.c"
                            "wifi_manager.c"
                            "ota_manager.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash esp_wifi esp_https_ota app_update esp_http_client
                               esp_netif esp_event bt esp_driver_gpio esp_driver_uart esp_timer
                               esp_driver_i2s esp_ringbuf)
