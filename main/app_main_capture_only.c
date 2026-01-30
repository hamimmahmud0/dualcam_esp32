/*
 * SPDX-FileCopyrightText: 2024
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define TAG "capture_only"

#define CAPTURE_DIR "/eMMC/capture"
#define CAPTURE_FRAME_COUNT 51781
#define CAPTURE_DROP_FRAMES 5
#define CAPTURE_SESSION "slavecam"

#define INIT_DELAY_MS 200

static void init_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static esp_err_t ensure_dir(const char *path)
{
    struct stat st;
    if (stat(path, &st) == 0) {
        return S_ISDIR(st.st_mode) ? ESP_OK : ESP_FAIL;
    }
    return (mkdir(path, 0775) == 0) ? ESP_OK : ESP_FAIL;
}

static esp_err_t mount_and_format_sdcard(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;

    sdmmc_card_t *card = NULL;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/eMMC", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Formatting SD card");
    ret = esp_vfs_fat_sdcard_format("/eMMC", card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD card format failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ensure_dir(CAPTURE_DIR);
}

static esp_err_t init_camera_rgb565(void)
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
        .pixel_format = PIXFORMAT_RGB565,
        .frame_size = FRAMESIZE_VGA,
        .jpeg_quality = 12,
        .fb_count = 2,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .fb_location = CAMERA_FB_IN_PSRAM,
    };

    if (esp_psram_is_initialized()) {
        ESP_LOGI(TAG, "PSRAM is initialized (free=%u)",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    } else {
        ESP_LOGW(TAG, "PSRAM is NOT initialized");
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        return err;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor && sensor->id.PID == OV3660_PID) {
        sensor->set_vflip(sensor, 1);
        sensor->set_brightness(sensor, 1);
        sensor->set_saturation(sensor, -2);
    }
    return ESP_OK;
}


// variables

camera_fb_t *fb_cap;
int64_t timestamp_ms;
char path[256];
int64_t delta_ms;


static esp_err_t capture_sequence(void)
{
    for (int i = 0; i < CAPTURE_DROP_FRAMES; ++i) {
        fb_cap = esp_camera_fb_get();
        if (fb_cap) {
            esp_camera_fb_return(fb_cap);
        }
    }

    int64_t prev_timestamp_ms = -1;
    for (int i = 0; i < CAPTURE_FRAME_COUNT; ++i) {
        fb_cap = esp_camera_fb_get();
        if (!fb_cap) {
            ESP_LOGW(TAG, "Frame capture failed (%d)", i);
            continue;
        }

        timestamp_ms = esp_timer_get_time() / 1000;
        
        snprintf(path, sizeof(path), "%s/%s-%lld.rgb565", CAPTURE_DIR, CAPTURE_SESSION,
                 (long long)timestamp_ms);
        delta_ms = (prev_timestamp_ms >= 0) ? (timestamp_ms - prev_timestamp_ms) : 0;
        ESP_LOGI(TAG, "path: %s (frame %d/%d, dt=%lldms)", path, i + 1, CAPTURE_FRAME_COUNT,
                 (long long)delta_ms);

        FILE *file = fopen(path, "wb");
        if (!file) {
            ESP_LOGW(TAG, "Failed to open %s", path);
            esp_camera_fb_return(fb_cap);
            continue;
        }
        fwrite(fb_cap->buf, 1, fb_cap->len, file);
        fclose(file);

        esp_camera_fb_return(fb_cap);
        prev_timestamp_ms = timestamp_ms;
    }

    return ESP_OK;
}

void app_main(void)
{
    init_delay_ms(INIT_DELAY_MS);

    ESP_ERROR_CHECK(mount_and_format_sdcard());
    init_delay_ms(INIT_DELAY_MS);

    ESP_ERROR_CHECK(init_camera_rgb565());
    init_delay_ms(INIT_DELAY_MS);

    ESP_ERROR_CHECK(capture_sequence());
    esp_camera_deinit();

    ESP_LOGI(TAG, "Capture complete");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
