/*
 * SPDX-FileCopyrightText: 2024
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_camera.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "esp_wifi.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#define TAG "udp_rgb565"

#if CONFIG_ENABLE_LOGGING
#define LOGI(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) ESP_LOGW(TAG, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) ESP_LOGE(TAG, fmt, ##__VA_ARGS__)
#else
#define LOGI(...) ((void)0)
#define LOGW(...) ((void)0)
#define LOGE(...) ((void)0)
#endif

#define STREAM_CMD_PORT 12500
#define STREAM_DATA_PORT 12501
#define UDP_PAYLOAD_MAX 1472

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define FRAME_SIZE_BYTES (FRAME_WIDTH * FRAME_HEIGHT * 2)

#define UDP_HEADER_SIZE 12
#define UDP_DATA_CHUNK (UDP_PAYLOAD_MAX - UDP_HEADER_SIZE)

#define CMD_START "START"
#define CMD_STOP "STOP"

#define UDP_SEND_RETRY_MAX 4
#define UDP_SEND_RETRY_DELAY_MS 2
#define UDP_SEND_PACE_EVERY_N 8
#define UDP_SEND_PACE_DELAY_MS 1

#define INIT_DELAY_MS 200

#if CONFIG_FREERTOS_UNICORE
#define CAPTURE_TASK_CORE 0
#define UDP_TASK_CORE 0
#else
#define CAPTURE_TASK_CORE 1
#define UDP_TASK_CORE 0
#endif

#define CAPTURE_TASK_STACK_SIZE 4096
#define UDP_TASK_STACK_SIZE 6144
#define CAPTURE_TASK_PRIORITY 5
#define UDP_TASK_PRIORITY 5

typedef struct __attribute__((packed)) {
    uint32_t frame_id;
    uint16_t packet_index;
    uint16_t packet_count;
    uint16_t payload_len;
    uint16_t reserved;
} udp_frame_header_t;

typedef struct {
    camera_fb_t *fb;
    uint32_t frame_id;
} frame_item_t;

static QueueHandle_t s_frame_queue = NULL;
static EventGroupHandle_t s_wifi_event_group = NULL;
static const int WIFI_CONNECTED_BIT = BIT0;

static volatile bool s_stream_enabled = false;
static volatile bool s_client_valid = false;
static struct sockaddr_in s_stream_client = {0};

static void init_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, CONFIG_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode =
        (strlen(CONFIG_WIFI_PASSWORD) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    return ESP_OK;
}

static esp_err_t init_mdns(void)
{
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("cam-calib"));
    ESP_ERROR_CHECK(mdns_instance_name_set("cam-calib"));
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_camstream", "_udp", STREAM_DATA_PORT, NULL, 0));
    return ESP_OK;
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
        .fb_count = 4,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .fb_location = CAMERA_FB_IN_PSRAM,
    };

    if (esp_psram_is_initialized()) {
        LOGI("PSRAM initialized (free=%u)",
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
    }
    return ESP_OK;
}

static void drain_frame_queue(void)
{
    if (!s_frame_queue) {
        return;
    }

    frame_item_t item;
    while (xQueueReceive(s_frame_queue, &item, 0) == pdTRUE) {
        if (item.fb) {
            esp_camera_fb_return(item.fb);
        }
    }
}

static esp_err_t udp_send_frame(int sock, const struct sockaddr_in *dest, const frame_item_t *item)
{
    static TickType_t s_last_send_err_tick = 0;
    if (!dest || !item || !item->fb) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t *data = item->fb->buf;
    size_t frame_len = item->fb->len;
    if (frame_len < FRAME_SIZE_BYTES) {
        LOGW("Frame too small: %u bytes", (unsigned)frame_len);
        return ESP_ERR_INVALID_SIZE;
    }
    if (frame_len > FRAME_SIZE_BYTES) {
        frame_len = FRAME_SIZE_BYTES;
    }

    const uint16_t packet_count = (frame_len + UDP_DATA_CHUNK - 1) / UDP_DATA_CHUNK;

    uint8_t packet[UDP_PAYLOAD_MAX];
    for (uint16_t idx = 0; idx < packet_count; ++idx) {
        if (!s_stream_enabled) {
            return ESP_OK;
        }
        size_t offset = (size_t)idx * UDP_DATA_CHUNK;
        size_t chunk = frame_len - offset;
        if (chunk > UDP_DATA_CHUNK) {
            chunk = UDP_DATA_CHUNK;
        }

        udp_frame_header_t header = {
            .frame_id = item->frame_id,
            .packet_index = idx,
            .packet_count = packet_count,
            .payload_len = (uint16_t)chunk,
            .reserved = 0,
        };

        memcpy(packet, &header, sizeof(header));
        memcpy(packet + sizeof(header), data + offset, chunk);

        int sent = -1;
        int send_errno = 0;
        for (int attempt = 0; attempt < UDP_SEND_RETRY_MAX; ++attempt) {
            sent = sendto(sock, packet, sizeof(header) + chunk, 0,
                          (const struct sockaddr *)dest, sizeof(*dest));
            if (sent >= 0) {
                break;
            }
            send_errno = errno;
            if (send_errno == ENOMEM || send_errno == ENOBUFS || send_errno == EAGAIN) {
                vTaskDelay(pdMS_TO_TICKS(UDP_SEND_RETRY_DELAY_MS));
                continue;
            }
            break;
        }
        if (sent < 0) {
            TickType_t now_tick = xTaskGetTickCount();
            if (now_tick - s_last_send_err_tick > pdMS_TO_TICKS(1000)) {
                LOGW("UDP send failed: errno=%d", send_errno);
                s_last_send_err_tick = now_tick;
            }
            return ESP_FAIL;
        }

        if ((idx % UDP_SEND_PACE_EVERY_N) == 0) {
            vTaskDelay(pdMS_TO_TICKS(UDP_SEND_PACE_DELAY_MS));
        }
    }

    return ESP_OK;
}

static void capture_task(void *arg)
{
    (void)arg;
    uint32_t frame_id = 0;

    for (;;) {
        if (!s_stream_enabled) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            LOGW("Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (fb->format != PIXFORMAT_RGB565) {
            LOGW("Unexpected format %d", fb->format);
            esp_camera_fb_return(fb);
            continue;
        }

        frame_item_t item = {
            .fb = fb,
            .frame_id = frame_id++,
        };

        if (xQueueSend(s_frame_queue, &item, 0) != pdTRUE) {
            esp_camera_fb_return(fb);
        }
    }
}

static void udp_stream_task(void *arg)
{
    (void)arg;

    int ctrl_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (ctrl_sock < 0) {
        LOGE("Control socket create failed: errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in ctrl_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(STREAM_CMD_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(ctrl_sock, (struct sockaddr *)&ctrl_addr, sizeof(ctrl_addr)) < 0) {
        LOGE("Control socket bind failed: errno=%d", errno);
        close(ctrl_sock);
        vTaskDelete(NULL);
        return;
    }

    int stream_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (stream_sock < 0) {
        LOGE("Stream socket create failed: errno=%d", errno);
        close(ctrl_sock);
        vTaskDelete(NULL);
        return;
    }
    int snd_buf = 64 * 1024;
    setsockopt(stream_sock, SOL_SOCKET, SO_SNDBUF, &snd_buf, sizeof(snd_buf));

    struct sockaddr_in stream_bind = {
        .sin_family = AF_INET,
        .sin_port = htons(STREAM_DATA_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(stream_sock, (struct sockaddr *)&stream_bind, sizeof(stream_bind)) < 0) {
        LOGE("Stream socket bind failed: errno=%d", errno);
        close(stream_sock);
        close(ctrl_sock);
        vTaskDelete(NULL);
        return;
    }

    struct timeval timeout = {
        .tv_sec = 0,
        .tv_usec = 100 * 1000,
    };
    setsockopt(ctrl_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    char rx_buf[64];
    for (;;) {
        struct sockaddr_storage source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(ctrl_sock, rx_buf, sizeof(rx_buf) - 1, 0,
                           (struct sockaddr *)&source_addr, &socklen);
        if (len > 0) {
            rx_buf[len] = '\0';
            if (strncmp(rx_buf, CMD_START, strlen(CMD_START)) == 0) {
                if (source_addr.ss_family == AF_INET) {
                    memcpy(&s_stream_client, &source_addr, sizeof(struct sockaddr_in));
                    s_stream_client.sin_port = htons(STREAM_DATA_PORT);
                    s_stream_enabled = true;
                    s_client_valid = true;
                    drain_frame_queue();
                    LOGI("Streaming enabled to %s:%u",
                         inet_ntoa(s_stream_client.sin_addr), STREAM_DATA_PORT);
                    const char *resp = "OK";
                    sendto(ctrl_sock, resp, strlen(resp), 0,
                           (struct sockaddr *)&source_addr, socklen);
                }
            } else if (strncmp(rx_buf, CMD_STOP, strlen(CMD_STOP)) == 0) {
                s_stream_enabled = false;
                s_client_valid = false;
                drain_frame_queue();
                const char *resp = "OK";
                sendto(ctrl_sock, resp, strlen(resp), 0,
                       (struct sockaddr *)&source_addr, socklen);
                LOGI("Streaming disabled");
            } else {
                const char *resp = "ERR";
                sendto(ctrl_sock, resp, strlen(resp), 0,
                       (struct sockaddr *)&source_addr, socklen);
            }
        }

        if (s_stream_enabled && s_client_valid) {
            frame_item_t item;
            if (xQueueReceive(s_frame_queue, &item, 0) == pdTRUE) {
                udp_send_frame(stream_sock, &s_stream_client, &item);
                if (item.fb) {
                    esp_camera_fb_return(item.fb);
                }
            }
        }
    }
}

static esp_err_t init_tasks(void)
{
    s_frame_queue = xQueueCreate(2, sizeof(frame_item_t));
    if (!s_frame_queue) {
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(capture_task, "capture", CAPTURE_TASK_STACK_SIZE,
                                            NULL, CAPTURE_TASK_PRIORITY, NULL, CAPTURE_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_FAIL;
    }

    ok = xTaskCreatePinnedToCore(udp_stream_task, "udp_stream", UDP_TASK_STACK_SIZE,
                                 NULL, UDP_TASK_PRIORITY, NULL, UDP_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    init_delay_ms(INIT_DELAY_MS);

    ESP_ERROR_CHECK(wifi_init());
    init_delay_ms(INIT_DELAY_MS);

    ESP_ERROR_CHECK(init_mdns());
    init_delay_ms(INIT_DELAY_MS);

    ESP_ERROR_CHECK(init_camera_rgb565());
    init_delay_ms(INIT_DELAY_MS);

    ESP_ERROR_CHECK(init_tasks());
    init_delay_ms(INIT_DELAY_MS);

    LOGI("UDP RGB565 streaming ready: cam-calib.local (cmd %d, stream %d)",
         STREAM_CMD_PORT, STREAM_DATA_PORT);
}
