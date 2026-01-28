/*
 * SPDX-FileCopyrightText: 2024
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <sys/time.h>
#include <ctype.h>
#include <errno.h>

#include "esp_psram.h"
#include "esp_heap_caps.h"


#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_netif.h"
#include "esp_rom_sys.h"
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include "esp_wifi.h"
#include "cJSON.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#define TAG "mastercam"

#define STREAM_BOUNDARY "123456789000000000000987654321"
#define STREAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY

#define DEFAULT_FRAME_SIZE FRAMESIZE_VGA
#define DEFAULT_PIXEL_FORMAT PIXFORMAT_JPEG

#define CAPTURE_DIR "/eMMC/capture"

#if CONFIG_FREERTOS_UNICORE
#define NET_TASK_CORE 0
#define CAPTURE_TASK_CORE 0
#else
#define NET_TASK_CORE 0
#define CAPTURE_TASK_CORE 1
#endif

#define CAPTURE_TASK_STACK_SIZE 8192
#define CAPTURE_TASK_PRIORITY 5

#ifndef HTTPD_409_CONFLICT
#define HTTPD_409_CONFLICT 409
#endif

#ifndef CONFIG_MASTER_ID
#define CONFIG_MASTER_ID "000000"
#endif
#ifndef CONFIG_SLAVE_ID
#define CONFIG_SLAVE_ID "000000"
#endif
#ifndef CONFIG_CAPSEQ_SLAVE_PREPARE_DELAY_MS
#define CONFIG_CAPSEQ_SLAVE_PREPARE_DELAY_MS 3000
#endif
#ifndef CONFIG_CAPSEQ_DROP_FRAMES
#define CONFIG_CAPSEQ_DROP_FRAMES 5
#endif
#ifndef CONFIG_CAPSEQ_SYNC_SAFETY_MS
#define CONFIG_CAPSEQ_SYNC_SAFETY_MS 1000
#endif
#ifndef CONFIG_CAPSEQ_SYNC_UDP_PINGS
#define CONFIG_CAPSEQ_SYNC_UDP_PINGS 5
#endif
#ifndef CONFIG_CAPSEQ_SYNC_UDP_PORT
#define CONFIG_CAPSEQ_SYNC_UDP_PORT 65
#endif
#ifndef CONFIG_CAPSEQ_ALLOW_SLAVE_MISSING
#define CONFIG_CAPSEQ_ALLOW_SLAVE_MISSING 0
#endif
#ifndef CONFIG_CAPSEQ_SLAVE_READY_TIMEOUT_MS
#define CONFIG_CAPSEQ_SLAVE_READY_TIMEOUT_MS 5000
#endif
#ifndef CONFIG_CAPSEQ_SLAVE_READY_POLL_MS
#define CONFIG_CAPSEQ_SLAVE_READY_POLL_MS 200
#endif
#ifndef CONFIG_CAPSEQ_SYNC_START_RETRIES
#define CONFIG_CAPSEQ_SYNC_START_RETRIES 3
#endif
#ifndef CONFIG_CAPSEQ_SYNC_START_RETRY_DELAY_MS
#define CONFIG_CAPSEQ_SYNC_START_RETRY_DELAY_MS 100
#endif

#if defined(SLAVE_NOT_AVAILAVLE)
#define CAPSEQ_SLAVE_MISSING_OK 1
#else
#define CAPSEQ_SLAVE_MISSING_OK CONFIG_CAPSEQ_ALLOW_SLAVE_MISSING
#endif

#define INIT_DELAY_MS 200
#define WIFI_POST_INIT_DELAY_MS 500
#define CAMERA_PWDN_PIN 32
#define CAMERA_RESET_DELAY_MS 20

static httpd_handle_t s_httpd = NULL;
static httpd_handle_t s_stream_httpd = NULL;
static volatile bool s_stream_enabled = false;
static volatile bool s_stream_in_progress = false;
static volatile bool s_stream_stop_requested = false;
static int s_stream_fd = -1;
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;
static QueueHandle_t s_capture_queue = NULL;

typedef struct {
    char session[32];
    char query[256];
    int frame_count;
    framesize_t fs;
    pixformat_t fmt;
    int64_t cpu_time_to_start_us;
    esp_err_t result;
    char err_msg[64];
    SemaphoreHandle_t done;
} capture_request_t;

typedef struct {
    int64_t trip_time_us;
    int64_t cpu_disparity_us;
} capseq_sync_metrics_t;

static esp_err_t init_camera(void);
static esp_err_t init_camera_with_format(framesize_t fs, pixformat_t pf);
static esp_err_t init_capture_task(void);
static void capture_task(void *arg);
static esp_err_t run_capture_sequence(capture_request_t *req);
static esp_err_t send_slave_stream_cmd(const char *path);
static esp_err_t udp_slave_wait_ready(int timeout_ms, int poll_ms);
static esp_err_t udp_slave_start_with_retry(int64_t start_delay_us);
static esp_err_t udp_slave_ready_check(void);
static esp_err_t udp_sync_metrics(capseq_sync_metrics_t *metrics);
static esp_err_t udp_slave_start_capture(int64_t start_delay_us);

static void camera_power_cycle(void)
{
    if (CAMERA_PWDN_PIN < 0) {
        return;
    }
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << CAMERA_PWDN_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(CAMERA_PWDN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(CAMERA_RESET_DELAY_MS));
    gpio_set_level(CAMERA_PWDN_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(CAMERA_RESET_DELAY_MS));
}

static void stop_stream_and_wait(uint32_t timeout_ms)
{
    s_stream_enabled = false;
    s_stream_stop_requested = true;
    if (s_stream_httpd != NULL && s_stream_fd >= 0) {
        httpd_sess_trigger_close(s_stream_httpd, s_stream_fd);
        s_stream_fd = -1;
    }
    uint32_t waited_ms = 0;
    while (s_stream_in_progress && waited_ms < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(20));
        waited_ms += 20;
    }
}

static void init_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void check_heap_integrity(const char *stage)
{
    if (!heap_caps_check_integrity_all(true)) {
        ESP_LOGE(TAG, "Heap corruption detected after %s", stage);
        abort();
    }
}

static esp_err_t ensure_dir(const char *path)
{
    struct stat st;
    if (stat(path, &st) == 0) {
        if (S_ISDIR(st.st_mode)) {
            return ESP_OK;
        }
        return ESP_FAIL;
    }

    if (mkdir(path, 0775) != 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static framesize_t parse_framesize(const char *value)
{
    if (!value) {
        return DEFAULT_FRAME_SIZE;
    }
    if (isdigit((unsigned char)value[0])) {
        return (framesize_t)atoi(value);
    }
    if (strcasecmp(value, "qqvga") == 0) return FRAMESIZE_QQVGA;
    if (strcasecmp(value, "qvga") == 0) return FRAMESIZE_QVGA;
    if (strcasecmp(value, "vga") == 0) return FRAMESIZE_VGA;
    if (strcasecmp(value, "svga") == 0) return FRAMESIZE_SVGA;
    if (strcasecmp(value, "xga") == 0) return FRAMESIZE_XGA;
    if (strcasecmp(value, "sxga") == 0) return FRAMESIZE_SXGA;
    if (strcasecmp(value, "uxga") == 0) return FRAMESIZE_UXGA;
    return DEFAULT_FRAME_SIZE;
}

static pixformat_t parse_pixformat(const char *value)
{
    if (!value) {
        return DEFAULT_PIXEL_FORMAT;
    }
    if (isdigit((unsigned char)value[0])) {
        return (pixformat_t)atoi(value);
    }
    if (strcasecmp(value, "jpeg") == 0) return PIXFORMAT_JPEG;
    if (strcasecmp(value, "rgb565") == 0) return PIXFORMAT_RGB565;
    if (strcasecmp(value, "grayscale") == 0) return PIXFORMAT_GRAYSCALE;
    if (strcasecmp(value, "yuv422") == 0) return PIXFORMAT_YUV422;
    return DEFAULT_PIXEL_FORMAT;
}

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void apply_sensor_setting(sensor_t *sensor, const char *key, int value)
{
    if (!sensor || !key) {
        return;
    }

    if (strcmp(key, "framesize") == 0) {
        sensor->set_framesize(sensor, clamp_int(value, 0, FRAMESIZE_UXGA));
    } else if (strcmp(key, "quality") == 0) {
        sensor->set_quality(sensor, clamp_int(value, 2, 63));
    } else if (strcmp(key, "brightness") == 0) {
        sensor->set_brightness(sensor, clamp_int(value, -2, 2));
    } else if (strcmp(key, "contrast") == 0) {
        sensor->set_contrast(sensor, clamp_int(value, -2, 2));
    } else if (strcmp(key, "saturation") == 0) {
        sensor->set_saturation(sensor, clamp_int(value, -2, 2));
    } else if (strcmp(key, "gainceiling") == 0) {
        sensor->set_gainceiling(sensor, clamp_int(value, 0, 6));
    } else if (strcmp(key, "colorbar") == 0) {
        sensor->set_colorbar(sensor, value ? 1 : 0);
    } else if (strcmp(key, "awb") == 0) {
        sensor->set_whitebal(sensor, value ? 1 : 0);
    } else if (strcmp(key, "awb_gain") == 0) {
        sensor->set_awb_gain(sensor, value ? 1 : 0);
    } else if (strcmp(key, "wb_mode") == 0) {
        sensor->set_wb_mode(sensor, clamp_int(value, 0, 4));
    } else if (strcmp(key, "aec2") == 0) {
        sensor->set_aec2(sensor, value ? 1 : 0);
    } else if (strcmp(key, "ae_level") == 0) {
        sensor->set_ae_level(sensor, clamp_int(value, -2, 2));
    } else if (strcmp(key, "aec_value") == 0) {
        sensor->set_aec_value(sensor, clamp_int(value, 0, 1200));
    } else if (strcmp(key, "agc") == 0) {
        sensor->set_gain_ctrl(sensor, value ? 1 : 0);
    } else if (strcmp(key, "agc_gain") == 0) {
        sensor->set_agc_gain(sensor, clamp_int(value, 0, 30));
    } else if (strcmp(key, "gain_ctrl") == 0) {
        sensor->set_gain_ctrl(sensor, value ? 1 : 0);
    } else if (strcmp(key, "bpc") == 0) {
        sensor->set_bpc(sensor, value ? 1 : 0);
    } else if (strcmp(key, "wpc") == 0) {
        sensor->set_wpc(sensor, value ? 1 : 0);
    } else if (strcmp(key, "raw_gma") == 0) {
        sensor->set_raw_gma(sensor, value ? 1 : 0);
    } else if (strcmp(key, "lenc") == 0) {
        sensor->set_lenc(sensor, value ? 1 : 0);
    } else if (strcmp(key, "hmirror") == 0) {
        sensor->set_hmirror(sensor, value ? 1 : 0);
    } else if (strcmp(key, "vflip") == 0) {
        sensor->set_vflip(sensor, value ? 1 : 0);
    } else if (strcmp(key, "dcw") == 0) {
        sensor->set_dcw(sensor, value ? 1 : 0);
    } else if (strcmp(key, "special_effect") == 0) {
        sensor->set_special_effect(sensor, clamp_int(value, 0, 6));
    } else if (strcmp(key, "exposure_ctrl") == 0) {
        sensor->set_exposure_ctrl(sensor, value ? 1 : 0);
    }
}

static void apply_sensor_settings_from_query_str(const char *query)
{
    if (!query || query[0] == '\0') {
        return;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor) {
        return;
    }

    char query_copy[256];
    snprintf(query_copy, sizeof(query_copy), "%s", query);

    char *saveptr = NULL;
    char *pair = strtok_r(query_copy, "&", &saveptr);
    while (pair) {
        char *eq = strchr(pair, '=');
        if (eq) {
            *eq = '\0';
            const char *key = pair;
            const char *value = eq + 1;
            if (strcmp(key, "framesize") == 0) {
                framesize_t fs = parse_framesize(value);
                apply_sensor_setting(sensor, "framesize", fs);
            } else if (strcmp(key, "pixel_format") == 0) {
                sensor->set_pixformat(sensor, parse_pixformat(value));
            } else {
                apply_sensor_setting(sensor, key, atoi(value));
            }
        }
        pair = strtok_r(NULL, "&", &saveptr);
    }
}

static esp_err_t read_body(httpd_req_t *req, char *buf, size_t buf_len)
{
    if (req->content_len >= buf_len) {
        return ESP_ERR_INVALID_SIZE;
    }
    size_t received = 0;
    while (received < req->content_len) {
        int len = httpd_req_recv(req, buf + received, req->content_len - received);
        if (len <= 0) {
            return ESP_FAIL;
        }
        received += len;
    }
    buf[received] = '\0';
    return ESP_OK;
}

static void apply_sensor_settings_from_json(sensor_t *sensor, const char *json_body)
{
    cJSON *root = cJSON_Parse(json_body);
    if (!root) {
        return;
    }

    const cJSON *item = NULL;
    cJSON_ArrayForEach(item, root) {
        if (!cJSON_IsString(item) && !cJSON_IsNumber(item) && !cJSON_IsBool(item)) {
            continue;
        }

        int value = 0;
        if (cJSON_IsNumber(item)) {
            value = (int)item->valuedouble;
        } else if (cJSON_IsBool(item)) {
            value = cJSON_IsTrue(item) ? 1 : 0;
        } else if (cJSON_IsString(item)) {
            if (strcmp(item->string, "framesize") == 0) {
                value = parse_framesize(item->valuestring);
            } else {
                value = atoi(item->valuestring);
            }
        }

        if (strcmp(item->string, "framesize") == 0) {
            apply_sensor_setting(sensor, "framesize", value);
        } else if (strcmp(item->string, "pixel_format") == 0) {
            pixformat_t pf = cJSON_IsString(item) ? parse_pixformat(item->valuestring) : (pixformat_t)value;
            sensor->set_pixformat(sensor, pf);
        } else {
            apply_sensor_setting(sensor, item->string, value);
        }
    }

    cJSON_Delete(root);
}

static esp_err_t home_handler(httpd_req_t *req)
{
    char path[64];
    snprintf(path, sizeof(path), "/www/index.html");

    FILE *file = fopen(path, "r");
    if (file) {
        httpd_resp_set_type(req, "text/html");
        char buf[256];
        size_t read_bytes;
        while ((read_bytes = fread(buf, 1, sizeof(buf), file)) > 0) {
            if (httpd_resp_send_chunk(req, buf, read_bytes) != ESP_OK) {
                fclose(file);
                httpd_resp_sendstr_chunk(req, NULL);
                return ESP_FAIL;
            }
        }
        fclose(file);
        httpd_resp_sendstr_chunk(req, NULL);
        return ESP_OK;
    }

    const char *fallback_html =
        "<!doctype html>"
        "<html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<title>MasterCam</title>"
        "<style>"
        "body{font-family:'Avenir Next',Arial,sans-serif;background:#f4f5f7;margin:0;color:#222;}"
        "header{background:#1f2937;color:#fff;padding:20px;}"
        "section{padding:20px;}"
        ".card{background:#fff;border-radius:8px;box-shadow:0 2px 8px rgba(0,0,0,.08);padding:16px;margin-bottom:16px;}"
        "button{background:#00d1b2;color:#fff;border:none;border-radius:4px;padding:10px 16px;margin:4px 4px 4px 0;}"
        "input,select{width:100%;padding:8px;margin-top:6px;border:1px solid #ddd;border-radius:4px;}"
        "</style></head><body>"
        "<header><h1>MasterCam</h1><p>ESP32-CAM Dual Sync</p></header>"
        "<section>"
        "<div class='card'>"
        "<button onclick=\"fetch('/api/stream/start')\">Start Stream</button>"
        "<button onclick=\"fetch('/api/stream/stop')\">Stop Stream</button>"
        "<button onclick=\"fetch('/api/capture?session=test&frame_count=1')\">Capture Sequence</button>"
        "</div>"
        "<div class='card'>"
        "<h3>Sensor Settings</h3>"
        "<form id='sensorForm'>"
        "<label>Framesize</label><select name='framesize'><option>svga</option><option>vga</option></select>"
        "<label>Quality</label><input name='quality' value='10'/>"
        "<button type='submit'>Apply</button>"
        "</form>"
        "</div>"
        "<div class='card'>"
        "<h3>Capture Sequence</h3>"
        "<form id='captureForm'>"
        "<label>Session</label><input name='session' value='session1'/>"
        "<label>Frame Count</label><input name='frame_count' value='5'/>"
        "<button type='submit'>Start</button>"
        "</form>"
        "</div>"
        "<div class='card'>"
        "<h3>Streams</h3>"
        "<p>Master</p><img id='stream' style='width:100%;max-width:640px;' />"
        "<p>Slave</p><img id='slaveStream' style='width:100%;max-width:640px;' />"
        "</div>"
        "</section>"
        "<script>"
        "document.getElementById('sensorForm').onsubmit=function(e){e.preventDefault();"
        "const params=new URLSearchParams(new FormData(this));fetch('/api/sensor',{method:'POST',body:params});};"
        "document.getElementById('captureForm').onsubmit=function(e){e.preventDefault();"
        "const params=new URLSearchParams(new FormData(this));fetch('/api/capture?'+params.toString());};"
        "const streamUrl=location.protocol+'//'+location.hostname+':81/stream';"
        "const slaveUrl=location.protocol+'//slavecam-" CONFIG_SLAVE_ID ".local:81/stream';"
        "document.getElementById('stream').src=streamUrl;"
        "document.getElementById('slaveStream').src=slaveUrl;"
        "</script></body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, fallback_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t stream_start_handler(httpd_req_t *req)
{
    s_stream_enabled = true;
    esp_err_t err = send_slave_stream_cmd("/api/stream/start");
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Slave stream start failed: %s", esp_err_to_name(err));
    }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t stream_stop_handler(httpd_req_t *req)
{
    s_stream_enabled = false;
    s_stream_stop_requested = true;
    if (s_stream_httpd != NULL && s_stream_fd >= 0) {
        httpd_sess_trigger_close(s_stream_httpd, s_stream_fd);
        s_stream_fd = -1;
    }
    esp_err_t err = send_slave_stream_cmd("/api/stream/stop");
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Slave stream stop failed: %s", esp_err_to_name(err));
    }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    int64_t uptime_ms = esp_timer_get_time() / 1000;
    uint32_t free_heap = esp_get_free_heap_size();
    char response[320];
    snprintf(response, sizeof(response),
             "{\"stream_enabled\":%s,\"stream_active\":%s,\"uptime_ms\":%lld,\"free_heap\":%" PRIu32
             ",\"slave_id\":\"%s\",\"master_id\":\"%s\"}",
             s_stream_enabled ? "true" : "false",
             s_stream_in_progress ? "true" : "false",
             uptime_ms,
             free_heap,
             CONFIG_SLAVE_ID,
             CONFIG_MASTER_ID);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    if (!s_stream_enabled) {
        httpd_resp_send_err(req, HTTPD_409_CONFLICT, "stream disabled");
        return ESP_FAIL;
    }
    if (s_stream_in_progress) {
        httpd_resp_send_err(req, HTTPD_409_CONFLICT, "stream already active");
        return ESP_FAIL;
    }

    s_stream_in_progress = true;
    s_stream_fd = httpd_req_to_sockfd(req);
    httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    char part_buf[128];

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        sensor->set_pixformat(sensor, PIXFORMAT_JPEG);
    }

    while (s_stream_enabled) {
        if (s_stream_stop_requested) {
            break;
        }
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Camera capture failed");
            continue;
        }

        int header_len = snprintf(part_buf, sizeof(part_buf),
                                  "--" STREAM_BOUNDARY "\r\n"
                                  "Content-Type: image/jpeg\r\n"
                                  "Content-Length: %u\r\n\r\n",
                                  fb->len);
        if (httpd_resp_send_chunk(req, part_buf, header_len) != ESP_OK ||
            httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len) != ESP_OK ||
            httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) {
            esp_camera_fb_return(fb);
            break;
        }

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    s_stream_in_progress = false;
    s_stream_stop_requested = false;
    if (s_stream_fd == httpd_req_to_sockfd(req)) {
        s_stream_fd = -1;
    }
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t sensor_handler(httpd_req_t *req)
{
    char content[512] = {0};
    if (read_body(req, content, sizeof(content)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid body");
        return ESP_FAIL;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "sensor not ready");
        return ESP_FAIL;
    }

    char type_buf[64] = {0};
    const char *content_type = type_buf;
    if (httpd_req_get_hdr_value_len(req, "Content-Type") > 0) {
        if (httpd_req_get_hdr_value_str(req, "Content-Type", type_buf, sizeof(type_buf)) == ESP_OK) {
            content_type = type_buf;
        }
    }

    if (strstr(content_type, "application/json") != NULL) {
        apply_sensor_settings_from_json(sensor, content);
    } else {
        char *saveptr;
        char *pair = strtok_r(content, "&", &saveptr);
        while (pair) {
            char *eq = strchr(pair, '=');
            if (eq) {
                *eq = '\0';
                const char *key = pair;
                const char *value = eq + 1;
                if (strcmp(key, "framesize") == 0) {
                    framesize_t fs = parse_framesize(value);
                    apply_sensor_setting(sensor, "framesize", fs);
                } else if (strcmp(key, "pixel_format") == 0) {
                    sensor->set_pixformat(sensor, parse_pixformat(value));
                } else {
                    apply_sensor_setting(sensor, key, atoi(value));
                }
            }
            pair = strtok_r(NULL, "&", &saveptr);
        }
    }

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

typedef struct {
    int sock;
    struct sockaddr_storage addr;
    socklen_t addrlen;
} udp_slave_ctx_t;

static bool parse_int64_payload(const char *buf, int64_t *out_value)
{
    if (!buf || !out_value) {
        return false;
    }
    char *end = NULL;
    errno = 0;
    int64_t value = strtoll(buf, &end, 10);
    if (errno != 0 || end == buf) {
        return false;
    }
    *out_value = value;
    return true;
}

static esp_err_t udp_open_slave_socket(udp_slave_ctx_t *ctx, int timeout_ms)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->sock = -1;

    char host[64];
    snprintf(host, sizeof(host), "slavecam-%s.local", CONFIG_SLAVE_ID);
    char port[8];
    snprintf(port, sizeof(port), "%d", CONFIG_CAPSEQ_SYNC_UDP_PORT);

    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_DGRAM,
    };
    struct addrinfo *res = NULL;
    int err = getaddrinfo(host, port, &hints, &res);
    if (err != 0 || !res) {
        ESP_LOGW(TAG, "UDP resolve failed for %s:%s (%d)", host, port, err);
        return ESP_FAIL;
    }

    ctx->sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (ctx->sock < 0) {
        ESP_LOGW(TAG, "UDP socket create failed (%d)", errno);
        freeaddrinfo(res);
        return ESP_FAIL;
    }

    memcpy(&ctx->addr, res->ai_addr, res->ai_addrlen);
    ctx->addrlen = res->ai_addrlen;
    freeaddrinfo(res);

    struct timeval tv = {
        .tv_sec = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000,
    };
    setsockopt(ctx->sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(ctx->sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    return ESP_OK;
}

static void udp_close_slave_socket(udp_slave_ctx_t *ctx)
{
    if (ctx && ctx->sock >= 0) {
        close(ctx->sock);
        ctx->sock = -1;
    }
}

static esp_err_t udp_send_and_recv(udp_slave_ctx_t *ctx, const char *payload, int payload_len,
                                   char *rx_buf, size_t rx_len, int *out_rx_len)
{
    if (!ctx || ctx->sock < 0 || !payload || payload_len <= 0 || !rx_buf || rx_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    int sent = sendto(ctx->sock, payload, payload_len, 0,
                      (struct sockaddr *)&ctx->addr, ctx->addrlen);
    if (sent < 0) {
        return ESP_FAIL;
    }
    struct sockaddr_storage from;
    socklen_t from_len = sizeof(from);
    int received = recvfrom(ctx->sock, rx_buf, rx_len - 1, 0,
                            (struct sockaddr *)&from, &from_len);
    if (received < 0) {
        return ESP_ERR_TIMEOUT;
    }
    rx_buf[received] = '\0';
    if (out_rx_len) {
        *out_rx_len = received;
    }
    return ESP_OK;
}

static esp_err_t udp_slave_ready_check(void)
{
    udp_slave_ctx_t ctx;
    if (udp_open_slave_socket(&ctx, 300) != ESP_OK) {
        return ESP_FAIL;
    }

    char rx_buf[64];
    esp_err_t err = udp_send_and_recv(&ctx, "READY", 5, rx_buf, sizeof(rx_buf), NULL);
    udp_close_slave_socket(&ctx);
    if (err != ESP_OK) {
        return err;
    }
    if (strncmp(rx_buf, "OK", 2) == 0) {
        return ESP_OK;
    }
    return ESP_FAIL;
}

static esp_err_t udp_slave_wait_ready(int timeout_ms, int poll_ms)
{
    int elapsed = 0;
    while (elapsed <= timeout_ms) {
        esp_err_t err = udp_slave_ready_check();
        if (err == ESP_OK) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(poll_ms));
        elapsed += poll_ms;
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t udp_sync_metrics(capseq_sync_metrics_t *metrics)
{
    if (!metrics) {
        return ESP_ERR_INVALID_ARG;
    }
    metrics->trip_time_us = 0;
    metrics->cpu_disparity_us = 0;

    udp_slave_ctx_t ctx;
    if (udp_open_slave_socket(&ctx, 300) != ESP_OK) {
        return ESP_FAIL;
    }

    int64_t rtt_sum = 0;
    int64_t disparity_sum = 0;
    int samples = 0;

    for (int i = 0; i < CONFIG_CAPSEQ_SYNC_UDP_PINGS; ++i) {
        int64_t send_time = esp_timer_get_time();
        char payload[32];
        int payload_len = snprintf(payload, sizeof(payload), "%lld", (long long)send_time);
        char rx_buf[64];
        esp_err_t err = udp_send_and_recv(&ctx, payload, payload_len, rx_buf, sizeof(rx_buf), NULL);
        int64_t recv_time = esp_timer_get_time();
        if (err != ESP_OK) {
            continue;
        }

        int64_t slave_time = 0;
        if (!parse_int64_payload(rx_buf, &slave_time)) {
            continue;
        }

        int64_t rtt_us = recv_time - send_time;
        int64_t master_at_slave = send_time + rtt_us / 2;
        int64_t disparity_us = master_at_slave - slave_time;
        rtt_sum += rtt_us;
        disparity_sum += disparity_us;
        samples++;
    }

    udp_close_slave_socket(&ctx);
    if (samples == 0) {
        return ESP_FAIL;
    }

    int64_t avg_rtt = rtt_sum / samples;
    metrics->trip_time_us = avg_rtt / 2;
    metrics->cpu_disparity_us = disparity_sum / samples;
    return ESP_OK;
}

static esp_err_t udp_slave_start_capture(int64_t start_delay_us)
{
    udp_slave_ctx_t ctx;
    if (udp_open_slave_socket(&ctx, 300) != ESP_OK) {
        return ESP_FAIL;
    }

    char payload[64];
    int payload_len = snprintf(payload, sizeof(payload), "START %lld", (long long)start_delay_us);
    char rx_buf[32];
    esp_err_t err = udp_send_and_recv(&ctx, payload, payload_len, rx_buf, sizeof(rx_buf), NULL);
    udp_close_slave_socket(&ctx);
    if (err != ESP_OK) {
        return err;
    }
    if (strncmp(rx_buf, "ACK", 3) == 0) {
        return ESP_OK;
    }
    return ESP_FAIL;
}

static esp_err_t udp_slave_start_with_retry(int64_t start_delay_us)
{
    for (int attempt = 0; attempt < CONFIG_CAPSEQ_SYNC_START_RETRIES; ++attempt) {
        esp_err_t err = udp_slave_start_capture(start_delay_us);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIG_CAPSEQ_SYNC_START_RETRY_DELAY_MS));
    }
    return ESP_FAIL;
}

static esp_err_t send_slave_prepare(const char *query)
{
    char url[128];
    snprintf(url, sizeof(url), "http://slavecam-%s.local/api/capture", CONFIG_SLAVE_ID);

    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 2000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
    if (query) {
        esp_http_client_set_post_field(client, query, strlen(query));
    }

    esp_err_t err = esp_http_client_perform(client);
    esp_http_client_cleanup(client);
    return err;
}

static esp_err_t send_slave_stream_cmd(const char *path)
{
    if (!path) {
        return ESP_ERR_INVALID_ARG;
    }
    char url[128];
    snprintf(url, sizeof(url), "http://slavecam-%s.local%s", CONFIG_SLAVE_ID, path);

    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 1000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_err_t err = esp_http_client_perform(client);
    esp_http_client_cleanup(client);
    return err;
}

static esp_err_t run_capture_sequence(capture_request_t *req)
{
    if (!req) {
        return ESP_ERR_INVALID_ARG;
    }

    req->err_msg[0] = '\0';
    stop_stream_and_wait(2000);

    esp_err_t err = send_slave_prepare(req->query[0] ? req->query : NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Slave prepare failed: %s", esp_err_to_name(err));
    }

    vTaskDelay(pdMS_TO_TICKS(CONFIG_CAPSEQ_SLAVE_PREPARE_DELAY_MS));

    bool need_reinit = true;
    esp_camera_deinit();
    gpio_uninstall_isr_service();
    camera_power_cycle();
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_err_t init_err = init_camera_with_format(req->fs, req->fmt);
    if (init_err != ESP_OK) {
        ESP_LOGW(TAG, "Capture camera init failed: %s", esp_err_to_name(init_err));
        snprintf(req->err_msg, sizeof(req->err_msg), "camera init failed");
        return ESP_FAIL;
    }

    apply_sensor_settings_from_query_str(req->query);

    for (int i = 0; i < CONFIG_CAPSEQ_DROP_FRAMES; ++i) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            esp_camera_fb_return(fb);
        }
    }

    bool slave_ready = true;
    esp_err_t sync_err = udp_slave_wait_ready(CONFIG_CAPSEQ_SLAVE_READY_TIMEOUT_MS,
                                               CONFIG_CAPSEQ_SLAVE_READY_POLL_MS);
    if (sync_err != ESP_OK) {
        ESP_LOGW(TAG, "Slave ready check failed");
        if (!CAPSEQ_SLAVE_MISSING_OK) {
            snprintf(req->err_msg, sizeof(req->err_msg), "slave not ready");
            return ESP_FAIL;
        }
        slave_ready = false;
    }

    capseq_sync_metrics_t metrics = {0};
    if (slave_ready) {
        sync_err = udp_sync_metrics(&metrics);
        if (sync_err != ESP_OK) {
            ESP_LOGW(TAG, "UDP sync failed");
            if (!CAPSEQ_SLAVE_MISSING_OK) {
                snprintf(req->err_msg, sizeof(req->err_msg), "udp sync failed");
                return ESP_FAIL;
            }
            slave_ready = false;
        }
    }

    int64_t safety_overhead_us = (req->cpu_time_to_start_us > 0)
                                     ? req->cpu_time_to_start_us
                                     : (int64_t)CONFIG_CAPSEQ_SYNC_SAFETY_MS * 1000;
    int64_t trip_time_us = slave_ready ? metrics.trip_time_us : 0;
    int64_t cpu_disparity_us = slave_ready ? metrics.cpu_disparity_us : 0;
    int64_t slave_start_delay_us = safety_overhead_us;
    int64_t master_start_delay_us = safety_overhead_us + trip_time_us + cpu_disparity_us;
    if (master_start_delay_us < 0) {
        master_start_delay_us = 0;
    }
    if (slave_ready) {
        ESP_LOGI(TAG, "Sync: trip=%lldus disparity=%lldus", (long long)trip_time_us,
                 (long long)cpu_disparity_us);
        sync_err = udp_slave_start_with_retry(slave_start_delay_us);
        if (sync_err != ESP_OK) {
            ESP_LOGW(TAG, "Slave start notify failed");
            if (!CAPSEQ_SLAVE_MISSING_OK) {
                snprintf(req->err_msg, sizeof(req->err_msg), "slave start failed");
                return ESP_FAIL;
            }
        }
    }

    int64_t start_time_us = esp_timer_get_time() + master_start_delay_us;
    while (true) {
        int64_t now_us = esp_timer_get_time();
        int64_t remaining_us = start_time_us - now_us;
        if (remaining_us <= 0) {
            break;
        }
        if (remaining_us > 2000) {
            vTaskDelay(pdMS_TO_TICKS(remaining_us / 1000));
        } else {
            esp_rom_delay_us(100);
        }
    }

    int64_t prev_timestamp_ms = -1;
    for (int i = 0; i < req->frame_count; ++i) {

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Frame capture failed (%d)", i);
            continue;
        }

        int64_t timestamp_ms = esp_timer_get_time() / 1000;
        char path[256];
        const char *ext = "session";
        if (req->fmt == PIXFORMAT_JPEG) ext = "jpg";
        else if (req->fmt == PIXFORMAT_RGB565) ext = "rgb565";
        else if (req->fmt == PIXFORMAT_GRAYSCALE) ext = "gray";
        else if (req->fmt == PIXFORMAT_YUV422) ext = "yuv";
        snprintf(path, sizeof(path), "%s/%s-%lld.%s", CAPTURE_DIR, req->session, timestamp_ms, ext);
        int64_t delta_ms = (prev_timestamp_ms >= 0) ? (timestamp_ms - prev_timestamp_ms) : 0;
        ESP_LOGI(TAG, "path: %s (frame %d/%d, dt=%lldms)", path, i + 1, req->frame_count,
                 (long long)delta_ms);
        FILE *file = fopen(path, "wb");
        if (!file) {
            ESP_LOGW(TAG, "Failed to open %s", path);
            esp_camera_fb_return(fb);
            continue;
        }
        fwrite(fb->buf, 1, fb->len, file);
        fclose(file);

        esp_camera_fb_return(fb);
        prev_timestamp_ms = timestamp_ms;
    }

    if (need_reinit) {
        esp_camera_deinit();
        gpio_uninstall_isr_service();
        camera_power_cycle();
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_err_t init_err = init_camera();
        if (init_err != ESP_OK) {
            ESP_LOGW(TAG, "Restore camera init failed: %s", esp_err_to_name(init_err));
        }
    }

    return ESP_OK;
}

static void capture_task(void *arg)
{
    (void)arg;
    capture_request_t *req = NULL;
    for (;;) {
        if (xQueueReceive(s_capture_queue, &req, portMAX_DELAY) == pdTRUE && req) {
            req->result = run_capture_sequence(req);
            if (req->done) {
                xSemaphoreGive(req->done);
            }
        }
    }
}

static esp_err_t init_capture_task(void)
{
    s_capture_queue = xQueueCreate(2, sizeof(capture_request_t *));
    if (!s_capture_queue) {
        return ESP_ERR_NO_MEM;
    }

    BaseType_t task_ok = xTaskCreatePinnedToCore(
        capture_task,
        "capture_task",
        CAPTURE_TASK_STACK_SIZE,
        NULL,
        CAPTURE_TASK_PRIORITY,
        NULL,
        CAPTURE_TASK_CORE);
    return (task_ok == pdPASS) ? ESP_OK : ESP_FAIL;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    char query[256] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing query");
        return ESP_FAIL;
    }

    char session[32] = {0};
    char value[32];
    int frame_count = 0;
    framesize_t fs = DEFAULT_FRAME_SIZE;
    pixformat_t fmt = DEFAULT_PIXEL_FORMAT;
    int64_t cpu_time_to_start_ms = -1;

    if (httpd_query_key_value(query, "session", session, sizeof(session)) != ESP_OK) {
        strncpy(session, "session", sizeof(session) - 1);
    }
    if (httpd_query_key_value(query, "frame_count", value, sizeof(value)) == ESP_OK) {
        frame_count = atoi(value);
    }
    if (frame_count <= 0) {
        frame_count = 1;
    }
    if (httpd_query_key_value(query, "framesize", value, sizeof(value)) == ESP_OK) {
        fs = parse_framesize(value);
    }
    if (httpd_query_key_value(query, "pixel_format", value, sizeof(value)) == ESP_OK) {
        fmt = parse_pixformat(value);
    }
    if (httpd_query_key_value(query, "cpu_time_to_start", value, sizeof(value)) == ESP_OK) {
        cpu_time_to_start_ms = atoll(value);
    }

    if (!s_capture_queue) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "capture task not ready");
        return ESP_FAIL;
    }

    capture_request_t *cap = calloc(1, sizeof(*cap));
    if (!cap) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
        return ESP_FAIL;
    }
    cap->done = xSemaphoreCreateBinary();
    if (!cap->done) {
        free(cap);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
        return ESP_FAIL;
    }

    snprintf(cap->session, sizeof(cap->session), "%s", session);
    snprintf(cap->query, sizeof(cap->query), "%s", query);
    cap->frame_count = frame_count;
    cap->fs = fs;
    cap->fmt = fmt;
    if (cpu_time_to_start_ms > 0) {
        cap->cpu_time_to_start_us = cpu_time_to_start_ms * 1000;
    }

    if (xQueueSend(s_capture_queue, &cap, pdMS_TO_TICKS(1000)) != pdTRUE) {
        vSemaphoreDelete(cap->done);
        free(cap);
        httpd_resp_send_err(req, HTTPD_409_CONFLICT, "capture busy");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(cap->done, portMAX_DELAY) != pdTRUE) {
        vSemaphoreDelete(cap->done);
        free(cap);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "capture timeout");
        return ESP_FAIL;
    }

    esp_err_t result = cap->result;
    const char *msg = cap->err_msg[0] ? cap->err_msg : "capture failed";
    vSemaphoreDelete(cap->done);
    free(cap);

    if (result != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, msg);
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
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

static esp_err_t mount_spiffs_www(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/www",
        .partition_label = "www",
        .max_files = 4,
        .format_if_mount_failed = false,
    };
    return esp_vfs_spiffs_register(&conf);
}

static esp_err_t mount_sdcard(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;

    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/eMMC", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }
    return ensure_dir(CAPTURE_DIR);
}

static esp_err_t init_camera(void)
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
        .frame_size = DEFAULT_FRAME_SIZE,
        .jpeg_quality = 12,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .fb_location = CAMERA_FB_IN_DRAM,
    };

    check_heap_integrity("before psram test");
    // Test PSRAM
    if (esp_psram_is_initialized()) {
        ESP_LOGI(TAG, "PSRAM is initialized");
        ESP_LOGI(TAG, "Total PSRAM size: %u bytes", (unsigned)esp_psram_get_size());

        // Try a simple allocation
        size_t test_size = 1024;
        void *test_ptr = heap_caps_malloc(test_size, MALLOC_CAP_SPIRAM);
        if (test_ptr) {
            ESP_LOGI(TAG, "Allocated %u bytes from PSRAM at %p", (unsigned)test_size, test_ptr);
            heap_caps_free(test_ptr);
        } else {
            ESP_LOGE(TAG, "Failed to allocate from PSRAM");
        }
    } else {
        ESP_LOGE(TAG, "PSRAM is NOT initialized");
    }

    ESP_LOGI(TAG, "PSRAM free before esp_camera_init: %u bytes",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
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

    if (sensor) {
        sensor->set_framesize(sensor, DEFAULT_FRAME_SIZE);
        sensor->set_pixformat(sensor, DEFAULT_PIXEL_FORMAT);
    }

    return ESP_OK;
}

static esp_err_t init_camera_with_format(framesize_t fs, pixformat_t pf)
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
        .pixel_format = pf,
        .frame_size = fs,
        .jpeg_quality = 12,
        .fb_count = 2,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .fb_location = CAMERA_FB_IN_PSRAM,
    };

    ESP_LOGI(TAG, "PSRAM free before esp_camera_init: %u bytes",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
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

static esp_err_t init_mdns(void)
{
    ESP_ERROR_CHECK(mdns_init());
    char hostname[32];
    snprintf(hostname, sizeof(hostname), "mastercam-%s", CONFIG_MASTER_ID);
    ESP_ERROR_CHECK(mdns_hostname_set(hostname));
    ESP_ERROR_CHECK(mdns_instance_name_set("MasterCam"));
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));
    return ESP_OK;
}

static esp_err_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 9;
    config.core_id = NET_TASK_CORE;

    if (httpd_start(&s_httpd, &config) != ESP_OK) {
        return ESP_FAIL;
    }

    httpd_uri_t home_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = home_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t capture_uri = {
        .uri = "/api/capture",
        .method = HTTP_GET,
        .handler = capture_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t sensor_uri = {
        .uri = "/api/sensor",
        .method = HTTP_POST,
        .handler = sensor_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t stream_start_uri = {
        .uri = "/api/stream/start",
        .method = HTTP_GET,
        .handler = stream_start_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t stream_stop_uri = {
        .uri = "/api/stream/stop",
        .method = HTTP_GET,
        .handler = stream_stop_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t status_uri = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = NULL,
    };

    httpd_register_uri_handler(s_httpd, &home_uri);
    httpd_register_uri_handler(s_httpd, &capture_uri);
    httpd_register_uri_handler(s_httpd, &sensor_uri);
    httpd_register_uri_handler(s_httpd, &stream_start_uri);
    httpd_register_uri_handler(s_httpd, &stream_stop_uri);
    httpd_register_uri_handler(s_httpd, &status_uri);

    return ESP_OK;
}

static esp_err_t start_stream_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 81;
    config.ctrl_port = 32769;
    config.max_uri_handlers = 1;
    config.core_id = NET_TASK_CORE;

    if (httpd_start(&s_stream_httpd, &config) != ESP_OK) {
        return ESP_FAIL;
    }

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL,
    };

    httpd_register_uri_handler(s_stream_httpd, &stream_uri);
    return ESP_OK;
}

void app_main(void)
{
    esp_log_set_vprintf(esp_rom_vprintf);
    check_heap_integrity("after log setup");
    ESP_ERROR_CHECK(nvs_flash_init());
    check_heap_integrity("after nvs_flash_init");
    init_delay_ms(INIT_DELAY_MS);

    check_heap_integrity("before mount_spiffs_www");
    if (mount_spiffs_www() != ESP_OK) {
        ESP_LOGW(TAG, "SPIFFS (www) not mounted");
    }
    check_heap_integrity("mount_spiffs_www");
    init_delay_ms(INIT_DELAY_MS);

    check_heap_integrity("before wifi_init");
    ESP_ERROR_CHECK(wifi_init());
    check_heap_integrity("wifi_init");
    init_delay_ms(WIFI_POST_INIT_DELAY_MS);
    check_heap_integrity("before init_mdns");
    ESP_ERROR_CHECK(init_mdns());
    check_heap_integrity("init_mdns");
    init_delay_ms(INIT_DELAY_MS);
    check_heap_integrity("before init_camera");
    ESP_ERROR_CHECK(init_camera());
    check_heap_integrity("init_camera");
    init_delay_ms(INIT_DELAY_MS);

    check_heap_integrity("before mount_sdcard");
    ESP_ERROR_CHECK(mount_sdcard());
    check_heap_integrity("mount_sdcard");
    init_delay_ms(INIT_DELAY_MS);
    check_heap_integrity("before init_capture_task");
    ESP_ERROR_CHECK(init_capture_task());
    check_heap_integrity("init_capture_task");
    init_delay_ms(INIT_DELAY_MS);
    check_heap_integrity("before start_webserver");
    ESP_ERROR_CHECK(start_webserver());
    check_heap_integrity("start_webserver");
    init_delay_ms(INIT_DELAY_MS);
    check_heap_integrity("before start_stream_server");
    ESP_ERROR_CHECK(start_stream_server());
    check_heap_integrity("start_stream_server");
    init_delay_ms(INIT_DELAY_MS);

    ESP_LOGI(TAG, "MasterCam ready: http://mastercam-%s.local/", CONFIG_MASTER_ID);
}
