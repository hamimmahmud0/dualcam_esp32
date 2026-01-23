#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_psram.h"
#include "esp_vfs_fat.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "driver/sdmmc_host.h"
#include "nvs_flash.h"
#include "sdmmc_cmd.h"
#include "esp_camera.h"

static const char *TAG = "psram_diag";
static const char *WIFI_TAG = "wifi_sta";
static const char *SD_TAG = "sdmmc";
static const char *CAM_TAG = "camera";

#define WIFI_SSID "Coolguys"
#define WIFI_PASS "4foolguys"
#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t s_wifi_event_group;

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(WIFI_TAG, "WiFi start, connecting to AP");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(WIFI_TAG, "Disconnected, retrying");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(WIFI_TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_wifi_event_group = xEventGroupCreate();
    if (!s_wifi_event_group) {
        ESP_LOGE(WIFI_TAG, "Failed to create event group");
        return;
    }

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void mount_sdcard(void)
{
    sdmmc_card_t *card = NULL;
    const char *mount_point = "/sdcard";
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;

    ESP_LOGI(SD_TAG, "Mounting SD card at %s (4-bit SDMMC)", mount_point);
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(SD_TAG, "Failed to mount filesystem. If card is unformatted, enable format_if_mount_failed");
        } else {
            ESP_LOGE(SD_TAG, "Failed to initialize SD card: %s", esp_err_to_name(ret));
        }
        return;
    }

    sdmmc_card_print_info(stdout, card);
}

static void init_camera_psram_svga(void)
{
    camera_config_t config = {
        .pin_pwdn = 32,
        .pin_reset = -1,
        .pin_xclk = 0,
        .pin_sccb_sda = 26,
        .pin_sccb_scl = 27,
        .pin_d7 = 35,
        .pin_d6 = 34,
        .pin_d5 = 39,
        .pin_d4 = 36,
        .pin_d3 = 21,
        .pin_d2 = 19,
        .pin_d1 = 18,
        .pin_d0 = 5,
        .pin_vsync = 25,
        .pin_href = 23,
        .pin_pclk = 22,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_SVGA,
        .jpeg_quality = 12,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };

    ESP_LOGI(CAM_TAG, "Initializing camera (SVGA, FB in PSRAM)");
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(CAM_TAG, "Camera init failed: %s", esp_err_to_name(err));
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s && s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_hmirror(s, 1);
    }
    ESP_LOGI(CAM_TAG, "Camera init OK");
}

static void psram_pattern_test(uint8_t *buf, size_t len, uint8_t pattern)
{
    memset(buf, pattern, len);
    for (size_t i = 0; i < len; ++i) {
        if (buf[i] != pattern) {
            ESP_LOGE(TAG, "Pattern mismatch at %u: got 0x%02x expected 0x%02x",
                     (unsigned)i, buf[i], pattern);
            return;
        }
    }
    ESP_LOGI(TAG, "Pattern 0x%02x OK for %u bytes", pattern, (unsigned)len);
}

void app_main(void)
{
    esp_log_set_vprintf(esp_rom_vprintf);
    ESP_LOGI(TAG, "PSRAM diag start");

#if !CONFIG_SPIRAM
    ESP_LOGE(TAG, "CONFIG_SPIRAM is disabled");
    return;
#endif

    if (!esp_psram_is_initialized()) {
        ESP_LOGW(TAG, "PSRAM not initialized by bootloader");
        esp_err_t err = esp_psram_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_psram_init failed: %s", esp_err_to_name(err));
            return;
        }
    }

    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "PSRAM free: %u bytes", (unsigned)psram_free);
    ESP_LOGI(TAG, "PSRAM largest block: %u bytes", (unsigned)psram_largest);

    if (CONFIG_SPIRAM_MEMTEST) {
        ESP_LOGW(TAG, "CONFIG_SPIRAM_MEMTEST enabled, but skipping esp_psram_extram_test");
        ESP_LOGW(TAG, "Reason: esp_psram_extram_test is destructive after PSRAM heap init");
    }

    const size_t test_size = 256 * 1024;
    uint8_t *buf = (uint8_t *)heap_caps_malloc(test_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        ESP_LOGE(TAG, "PSRAM alloc failed for %u bytes", (unsigned)test_size);
        return;
    }

    ESP_LOGI(TAG, "Allocated %u bytes in PSRAM", (unsigned)test_size);
    psram_pattern_test(buf, test_size, 0xAA);
    psram_pattern_test(buf, test_size, 0x55);

    heap_caps_free(buf);

    ESP_LOGI(TAG, "Starting WiFi STA after PSRAM tests");
    wifi_init_sta();

    mount_sdcard();

    init_camera_psram_svga();

    ESP_LOGI(TAG, "PSRAM diag done");
}
