/*
 * SPDX-FileCopyrightText: 2024
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <ctype.h>

#include "esp_psram.h"
#include "esp_heap_caps.h"


#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
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

#define TAG "mastercam"

#define STREAM_BOUNDARY "123456789000000000000987654321"
#define STREAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY

#define DEFAULT_FRAME_SIZE FRAMESIZE_SVGA
#define DEFAULT_PIXEL_FORMAT PIXFORMAT_JPEG

#define CAPTURE_DIR "/eMMC/capture"

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
#ifndef CONFIG_CAPSEQ_GPIO_TRIGGER
#define CONFIG_CAPSEQ_GPIO_TRIGGER 33
#endif

#define INIT_DELAY_MS 200
#define WIFI_POST_INIT_DELAY_MS 500

static httpd_handle_t s_httpd = NULL;
static volatile bool s_stream_enabled = false;
static volatile bool s_stream_in_progress = false;
static volatile bool s_stream_stop_requested = false;
static int s_stream_fd = -1;
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

static esp_err_t init_camera(void);
static esp_err_t init_camera_with_format(framesize_t fs, pixformat_t pf);

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
    if (strcasecmp(value, "raw") == 0) return PIXFORMAT_RAW;
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

static void apply_sensor_settings_from_query(httpd_req_t *req)
{
    char query[256] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor) {
        return;
    }

    char value[32];
    if (httpd_query_key_value(query, "framesize", value, sizeof(value)) == ESP_OK) {
        framesize_t fs = parse_framesize(value);
        apply_sensor_setting(sensor, "framesize", fs);
    }
    if (httpd_query_key_value(query, "quality", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "quality", atoi(value));
    }
    if (httpd_query_key_value(query, "brightness", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "brightness", atoi(value));
    }
    if (httpd_query_key_value(query, "contrast", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "contrast", atoi(value));
    }
    if (httpd_query_key_value(query, "saturation", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "saturation", atoi(value));
    }
    if (httpd_query_key_value(query, "gainceiling", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "gainceiling", atoi(value));
    }
    if (httpd_query_key_value(query, "colorbar", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "colorbar", atoi(value));
    }
    if (httpd_query_key_value(query, "awb", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "awb", atoi(value));
    }
    if (httpd_query_key_value(query, "awb_gain", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "awb_gain", atoi(value));
    }
    if (httpd_query_key_value(query, "wb_mode", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "wb_mode", atoi(value));
    }
    if (httpd_query_key_value(query, "aec2", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "aec2", atoi(value));
    }
    if (httpd_query_key_value(query, "ae_level", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "ae_level", atoi(value));
    }
    if (httpd_query_key_value(query, "aec_value", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "aec_value", atoi(value));
    }
    if (httpd_query_key_value(query, "agc", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "agc", atoi(value));
    }
    if (httpd_query_key_value(query, "agc_gain", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "agc_gain", atoi(value));
    }
    if (httpd_query_key_value(query, "gain_ctrl", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "gain_ctrl", atoi(value));
    }
    if (httpd_query_key_value(query, "bpc", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "bpc", atoi(value));
    }
    if (httpd_query_key_value(query, "wpc", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "wpc", atoi(value));
    }
    if (httpd_query_key_value(query, "raw_gma", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "raw_gma", atoi(value));
    }
    if (httpd_query_key_value(query, "lenc", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "lenc", atoi(value));
    }
    if (httpd_query_key_value(query, "hmirror", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "hmirror", atoi(value));
    }
    if (httpd_query_key_value(query, "vflip", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "vflip", atoi(value));
    }
    if (httpd_query_key_value(query, "dcw", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "dcw", atoi(value));
    }
    if (httpd_query_key_value(query, "special_effect", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "special_effect", atoi(value));
    }
    if (httpd_query_key_value(query, "exposure_ctrl", value, sizeof(value)) == ESP_OK) {
        apply_sensor_setting(sensor, "exposure_ctrl", atoi(value));
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
        "<h3>Stream</h3><img id='stream' style='width:100%;max-width:640px;' />"
        "</div>"
        "</section>"
        "<script>"
        "document.getElementById('sensorForm').onsubmit=function(e){e.preventDefault();"
        "const params=new URLSearchParams(new FormData(this));fetch('/api/sensor',{method:'POST',body:params});};"
        "document.getElementById('captureForm').onsubmit=function(e){e.preventDefault();"
        "const params=new URLSearchParams(new FormData(this));fetch('/api/capture?'+params.toString());};"
        "document.getElementById('stream').src='/stream';"
        "</script></body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, fallback_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t stream_start_handler(httpd_req_t *req)
{
    s_stream_enabled = true;
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t stream_stop_handler(httpd_req_t *req)
{
    s_stream_enabled = false;
    s_stream_stop_requested = true;
    if (s_httpd != NULL && s_stream_fd >= 0) {
        httpd_sess_trigger_close(s_httpd, s_stream_fd);
        s_stream_fd = -1;
    }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    int64_t uptime_ms = esp_timer_get_time() / 1000;
    uint32_t free_heap = esp_get_free_heap_size();
    char response[256];
    snprintf(response, sizeof(response),
             "{\"stream_enabled\":%s,\"stream_active\":%s,\"uptime_ms\":%lld,\"free_heap\":%" PRIu32 "}",
             s_stream_enabled ? "true" : "false",
             s_stream_in_progress ? "true" : "false",
             uptime_ms,
             free_heap);
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

static esp_err_t send_slave_prepare(const char *query)
{
    char url[128];
    snprintf(url, sizeof(url), "http://slavecam-%s.local/capture", CONFIG_SLAVE_ID);

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

    s_stream_enabled = false;

    esp_err_t err = send_slave_prepare(query);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Slave prepare failed: %s", esp_err_to_name(err));
    }

    vTaskDelay(pdMS_TO_TICKS(CONFIG_CAPSEQ_SLAVE_PREPARE_DELAY_MS));

    bool need_reinit = (fmt != PIXFORMAT_JPEG);
    if (need_reinit) {
        esp_camera_deinit();
        gpio_uninstall_isr_service();
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_err_t init_err = init_camera_with_format(fs, fmt);
        if (init_err != ESP_OK) {
            ESP_LOGW(TAG, "Capture camera init failed: %s", esp_err_to_name(init_err));
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "camera init failed");
            return ESP_FAIL;
        }
    } else {
        sensor_t *sensor = esp_camera_sensor_get();
        if (sensor) {
            sensor->set_framesize(sensor, fs);
            sensor->set_pixformat(sensor, fmt);
        }
    }

    apply_sensor_settings_from_query(req);

    for (int i = 0; i < CONFIG_CAPSEQ_DROP_FRAMES; ++i) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            esp_camera_fb_return(fb);
        }
    }

    gpio_set_level(CONFIG_CAPSEQ_GPIO_TRIGGER, 0);

    for (int i = 0; i < frame_count; ++i) {
        gpio_set_level(CONFIG_CAPSEQ_GPIO_TRIGGER, 1);
        esp_rom_delay_us(200);
        gpio_set_level(CONFIG_CAPSEQ_GPIO_TRIGGER, 0);

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Frame capture failed (%d)", i);
            continue;
        }

        int64_t timestamp_ms = esp_timer_get_time() / 1000;
        char path[256];
        const char *ext = "session";
        if (fmt == PIXFORMAT_JPEG) ext = "jpg";
        else if (fmt == PIXFORMAT_RGB565) ext = "rgb565";
        else if (fmt == PIXFORMAT_RAW) ext = "raw";
        else if (fmt == PIXFORMAT_GRAYSCALE) ext = "gray";
        else if (fmt == PIXFORMAT_YUV422) ext = "yuv";
        snprintf(path, sizeof(path), "%s/%s-%lld.%s", CAPTURE_DIR, session, timestamp_ms, ext);
        ESP_LOGI(TAG, "path: %s", path);
        FILE *file = fopen(path, "wb");
        if (!file) {
            ESP_LOGW(TAG, "Failed to open %s", path);
            esp_camera_fb_return(fb);
            continue;
        }
        fwrite(fb->buf, 1, fb->len, file);
        fclose(file);

        esp_camera_fb_return(fb);
    }

    if (need_reinit) {
        esp_camera_deinit();
        gpio_uninstall_isr_service();
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_err_t init_err = init_camera();
        if (init_err != ESP_OK) {
            ESP_LOGW(TAG, "Restore camera init failed: %s", esp_err_to_name(init_err));
        }
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
        .frame_size = FRAMESIZE_VGA,
        .jpeg_quality = 12,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        //.fb_location = CAMERA_FB_IN_PSRAM,
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
        .fb_count = 1,
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
    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
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
    httpd_register_uri_handler(s_httpd, &stream_uri);
    httpd_register_uri_handler(s_httpd, &stream_start_uri);
    httpd_register_uri_handler(s_httpd, &stream_stop_uri);
    httpd_register_uri_handler(s_httpd, &status_uri);

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

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << CONFIG_CAPSEQ_GPIO_TRIGGER,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    check_heap_integrity("before gpio_config");
    gpio_config(&io_conf);
    check_heap_integrity("gpio_config");
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
    check_heap_integrity("before start_webserver");
    ESP_ERROR_CHECK(start_webserver());
    check_heap_integrity("start_webserver");
    init_delay_ms(INIT_DELAY_MS);

    ESP_LOGI(TAG, "MasterCam ready: http://mastercam-%s.local/", CONFIG_MASTER_ID);
}
