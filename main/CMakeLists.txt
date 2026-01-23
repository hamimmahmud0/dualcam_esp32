idf_component_register(SRCS "app_main.c"
                       PRIV_REQUIRES esp_http_server esp_http_client esp_wifi nvs_flash mdns fatfs spiffs esp_timer driver esp32-camera esp_psram json
                       INCLUDE_DIRS "")

spiffs_create_partition_image(www ../main/www FLASH_IN_PROJECT)
