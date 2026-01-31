/*
 * SPDX-FileCopyrightText: 2024
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <sys/time.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

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

#if CONFIG_ENABLE_LOGGING
#define LOGI(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) ESP_LOGW(TAG, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) ESP_LOGE(TAG, fmt, ##__VA_ARGS__)
#else
#define LOGI(...) ((void)0)
#define LOGW(...) ((void)0)
#define LOGE(...) ((void)0)
#endif

#define CAPTURE_DIR "/eMMC/capture"
#define CAPTURE_DROP_FRAMES 5
#define CAPTURE_SESSION "slavecam"
#define CAPTURE_FILE_EXT ".frames"

#define INIT_DELAY_MS 200
#define CAPTURE_INTERVAL_MS 250
#define FRAME_QUEUE_LENGTH 30
#define CAPTURE_TASK_STACK_SIZE 4096
#define WRITER_TASK_STACK_SIZE 6144
#define CAPTURE_TASK_PRIORITY 5
#define WRITER_TASK_PRIORITY 5
#define CAPTURE_TASK_CORE 0
#define WRITER_TASK_CORE 1

#define SDCARD_SIZE_BYTES (8ULL * 1000ULL * 1000ULL * 1000ULL)
#define SDCARD_USABLE_BYTES (SDCARD_SIZE_BYTES * 9ULL / 10ULL)

typedef struct __attribute__((packed)) {
    uint64_t timestamp_ms;
    uint32_t data_len;
    uint16_t width;
    uint16_t height;
    uint8_t format;
    uint8_t reserved[3];
} frame_header_t;

static void log_shutter_time(sensor_t *sensor, int aec_value)
{
    int hts_h = sensor->get_reg(sensor, 0x380C, 0xFF);
    int hts_l = sensor->get_reg(sensor, 0x380D, 0xFF);
    if (hts_h < 0 || hts_l < 0) {
        LOGW("Failed to read HTS registers");
        return;
    }

    uint16_t hts = (uint16_t)((hts_h << 8) | hts_l);
    if (hts == 0 || CONFIG_CAPTURE_PCLK_HZ == 0) {
        LOGW("Invalid HTS/PCLK for shutter calc (hts=%u pclk=%d)",
             (unsigned)hts, CONFIG_CAPTURE_PCLK_HZ);
        return;
    }

    uint64_t line_time_us = ((uint64_t)hts * 1000000ULL) / (uint64_t)CONFIG_CAPTURE_PCLK_HZ;
    uint64_t shutter_us = ((uint64_t)aec_value * line_time_us) / 16ULL;
    LOGI("HTS=%u PCLK=%dHz line=%lluus shutter=%lluus (aec=%d)",
         (unsigned)hts, CONFIG_CAPTURE_PCLK_HZ,
         (unsigned long long)line_time_us,
         (unsigned long long)shutter_us,
         aec_value);
}

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
        .max_files = 5,
        .allocation_unit_size = 128 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;

    sdmmc_card_t *card = NULL;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/eMMC", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        LOGE("Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    LOGI("Formatting SD card");
    ret = esp_vfs_fat_sdcard_format("/eMMC", card);
    if (ret != ESP_OK) {
        LOGE("SD card format failed: %s", esp_err_to_name(ret));
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
        .frame_size = FRAMESIZE_QVGA,
        .jpeg_quality = 12,
        .fb_count = 5,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .fb_location = CAMERA_FB_IN_PSRAM,
    };

    if (esp_psram_is_initialized()) {
        LOGI("PSRAM is initialized (free=%u)",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    } else {
        LOGW("PSRAM is NOT initialized");
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
#if CONFIG_CAPTURE_MANUAL_EXPOSURE
        sensor->set_exposure_ctrl(sensor, 0);
        sensor->set_aec2(sensor, 0);
        sensor->set_aec_value(sensor, CONFIG_CAPTURE_MANUAL_EXPOSURE_VALUE);
        log_shutter_time(sensor, CONFIG_CAPTURE_MANUAL_EXPOSURE_VALUE);
#else
        log_shutter_time(sensor, sensor->status.aec_value);
#endif
    }
    return ESP_OK;
}


static QueueHandle_t frameQueue;
static volatile bool stop_capture;
static uint32_t max_frames;

static void capture_task(void *arg)
{
    (void)arg;

    if (stop_capture) {
        vTaskDelete(NULL);
    }

    for (int i = 0; i < CAPTURE_DROP_FRAMES; ++i) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            esp_camera_fb_return(fb);
        }
    }

    for (;;) {
        if (stop_capture) {
            vTaskDelete(NULL);
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            LOGW("Frame capture failed");
            vTaskDelay(pdMS_TO_TICKS(CAPTURE_INTERVAL_MS));
            continue;
        }

        if (stop_capture) {
            esp_camera_fb_return(fb);
            vTaskDelete(NULL);
        }

        if (xQueueSend(frameQueue, &fb, 0) != pdTRUE) {
            LOGW("Frame queue full, dropping frame");
            esp_camera_fb_return(fb);
        }

        vTaskDelay(pdMS_TO_TICKS(CAPTURE_INTERVAL_MS));
    }
}

static void writer_task(void *arg)
{
    (void)arg;

    char path[256];
    int64_t session_start_ms = esp_timer_get_time() / 1000;
    snprintf(path, sizeof(path), "%s/%s-%lld%s", CAPTURE_DIR, CAPTURE_SESSION,
             (long long)session_start_ms, CAPTURE_FILE_EXT);

    FILE *file = NULL;
    while (!file) {
        file = fopen(path, "wb");
        if (!file) {
            LOGW("Failed to open %s, retrying", path);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    LOGI("Writing frames to %s", path);

    int64_t prev_timestamp_ms = -1;
    uint32_t frame_index = 0;

    for (;;) {
        camera_fb_t *fb = NULL;
        if (xQueueReceive(frameQueue, &fb, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        int64_t timestamp_ms = esp_timer_get_time() / 1000;
        int64_t delta_ms = (prev_timestamp_ms >= 0) ? (timestamp_ms - prev_timestamp_ms) : 0;

        frame_header_t header = {
            .timestamp_ms = (uint64_t)timestamp_ms,
            .data_len = fb->len,
            .width = fb->width,
            .height = fb->height,
            .format = (uint8_t)fb->format,
            .reserved = {0},
        };

        if (max_frames == 0) {
            uint64_t frame_bytes = (uint64_t)sizeof(header) + (uint64_t)fb->len;
            max_frames = (uint32_t)(SDCARD_USABLE_BYTES / frame_bytes);
            if (max_frames == 0) {
                max_frames = 1;
            }
            LOGI("Frame bytes=%llu, max frames=%u",
                 (unsigned long long)frame_bytes, (unsigned)max_frames);
        }

        size_t header_written = fwrite(&header, 1, sizeof(header), file);
        int64_t write_start_us = esp_timer_get_time();
        size_t data_written = fwrite(fb->buf, 1, fb->len, file);
        int64_t write_end_us = esp_timer_get_time();

        if (header_written != sizeof(header) || data_written != fb->len) {
            LOGW("Short write (header=%u/%u data=%u/%u)",
                 (unsigned)header_written, (unsigned)sizeof(header),
                 (unsigned)data_written, (unsigned)fb->len);
        }
#if CONFIG_CAPTURE_FLUSH_EVERY_N_FRAMES > 0
        if ((frame_index % CONFIG_CAPTURE_FLUSH_EVERY_N_FRAMES) == 0) {
            fflush(file);
            fsync(fileno(file));
        }
#endif

        frame_index++;
        LOGI("frame %lu ts=%lldms dt=%lldms fwrite=%lld us (%u bytes)",
             (unsigned long)frame_index, (long long)timestamp_ms, (long long)delta_ms,
             (long long)(write_end_us - write_start_us),
             (unsigned)fb->len);
        esp_camera_fb_return(fb);
        prev_timestamp_ms = timestamp_ms;

        if (frame_index >= max_frames) {
            LOGI("Reached frame limit %u, stopping capture", (unsigned)max_frames);
            stop_capture = true;
            break;
        }
    }

    for (;;) {
        camera_fb_t *pending = NULL;
        if (xQueueReceive(frameQueue, &pending, 0) != pdTRUE) {
            break;
        }
        esp_camera_fb_return(pending);
    }

    fflush(file);
    fsync(fileno(file));
    fclose(file);
    vTaskDelete(NULL);
}

void app_main(void)
{
    init_delay_ms(INIT_DELAY_MS);

    ESP_ERROR_CHECK(mount_and_format_sdcard());
    init_delay_ms(INIT_DELAY_MS);

    ESP_ERROR_CHECK(init_camera_rgb565());
    init_delay_ms(INIT_DELAY_MS);

    frameQueue = xQueueCreate(FRAME_QUEUE_LENGTH, sizeof(camera_fb_t *));
    if (!frameQueue) {
        LOGE("Failed to create frame queue");
        return;
    }

    xTaskCreatePinnedToCore(capture_task, "capture_task", CAPTURE_TASK_STACK_SIZE, NULL,
                            CAPTURE_TASK_PRIORITY, NULL, CAPTURE_TASK_CORE);
    xTaskCreatePinnedToCore(writer_task, "writer_task", WRITER_TASK_STACK_SIZE, NULL,
                            WRITER_TASK_PRIORITY, NULL, WRITER_TASK_CORE);

    LOGI("Capture tasks started");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
