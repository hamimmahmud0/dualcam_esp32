if(CONFIG_APP_ROLE_SLAVE)
    set(APP_SRCS "app_main_slave.c")
else()
    set(APP_SRCS "app_main.c")
endif()

idf_component_register(SRCS ${APP_SRCS}
                       PRIV_REQUIRES esp_http_server esp_http_client esp_wifi nvs_flash mdns fatfs spiffs esp_timer driver esp32-camera esp_psram json
                       INCLUDE_DIRS "")

spiffs_create_partition_image(www ../main/www FLASH_IN_PROJECT)
