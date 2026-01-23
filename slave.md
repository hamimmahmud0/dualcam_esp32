# Slave Build Context Summary (ESP32-CAM Dual Sync)

This document summarizes the current “master” implementation and all relevant decisions, fixes, and runtime observations so you can build the **slave** variant. It is intentionally detailed for context transfer.

---

## 1) Target Hardware + Constraints

- **Board**: esp32cam-dwd0-v3 (revision 3.1)
- **Sensor**: OV3660
- **PSRAM**: 8MB (but PSRAM DMA issues seen; we now use DRAM for framebuffers)
- **Flash**: Boya (detected as generic; to optimize, enable `SPI_FLASH_SUPPORT_BOYA_CHIP` in menuconfig)
- **Flash size**: **4MB** (IMPORTANT: boot logs show detected size 4MB; earlier 8MB config caused assert)
- **SD card**: 8GB, 4-bit SDMMC, 52 MHz max
- **IDF**: v5.5

**Important runtime notes**:
- When configured as 8MB, boot asserted due to `Detected size(4096k) smaller than image header(8192k)`. We switched to 4MB partitioning.
- OV3660 is detected when correct AiThinker pin map is used.
- PSRAM DMA allocation failed for large JPEG buffers; solution was smaller initial framesize and DRAM framebuffers.

---

## 2) Project Structure

Workspace: `/home/hamim-mahmud/esp/esp-idf/hamim/new_from_codex`

Key files:
- `main/app_main.c` (all logic)
- `main/Kconfig.projbuild` (menu config: master/slave IDs, Wi-Fi, capture timing, GPIO trigger)
- `main/CMakeLists.txt`
- `partitions.csv` (custom partition table)
- `sdkconfig` (flash size = 4MB, custom partition table)
- `main/www/index.html` (SPIFFS UI)
- `main/idf_component.yml` (component manager dependencies)

The master implementation is a monolithic application. The slave should reuse most of the HTTP server/camera code but alter endpoints and capture triggering.

---

## 3) Camera Pin Map (AiThinker-style)

These were used to fix “camera not supported” and align with the known working reference:

```
PWDN  = 32
RESET = -1
XCLK  = 0
SIOD  = 26
SIOC  = 27
D7    = 35
D6    = 34
D5    = 39
D4    = 36
D3    = 21
D2    = 19
D1    = 18
D0    = 5
VSYNC = 25
HREF  = 23
PCLK  = 22
```

---

## 4) Master App Main Flow (Current Behavior)

Order in `app_main()`:
1. `nvs_flash_init()`
2. Mount SPIFFS (`/www` from partition label `www`)
3. **Wi‑Fi init** (with `esp_wifi_set_storage(WIFI_STORAGE_RAM)` to avoid NVS flash writes during camera DMA)
4. mDNS init (master hostname `mastercam-<id>.local`)
5. Mount SD card at `/eMMC`, ensure `/eMMC/capture`
6. Camera init
7. Start HTTP server

This order (Wi‑Fi first) avoids cache/DMA issues seen when Wi‑Fi writes to NVS while camera DMA is active.

---

## 5) Camera Init Strategy (Critical)

`init_camera()` sets:
- `PIXFORMAT_JPEG` initially
- `FRAMESIZE_VGA` for safety
- `jpeg_quality = 12`
- `fb_count = 1`
- `grab_mode = CAMERA_GRAB_WHEN_EMPTY`
- **`fb_location = CAMERA_FB_IN_DRAM`** (to avoid PSRAM DMA allocation failures)

Then:
- If OV3660: set vflip=1, brightness=1, saturation=-2
- After init, sets framesize + pixformat to defaults (`DEFAULT_FRAME_SIZE`, `DEFAULT_PIXEL_FORMAT`).

**Default pixel format** is now JPEG to avoid “NO-SOI” errors in cam_hal.

---

## 6) Runtime JPEG vs RGB565 Problem and Fix

Observed error:
```
cam_hal: NO-SOI - JPEG start marker missing
...
```
Cause: camera HAL remained in JPEG mode while sensor was set to RGB565; only changing `sensor->set_pixformat()` isn’t enough because DMA pipeline is still configured for JPEG.

**Fix implemented:**
- In capture endpoint, if `pixel_format != JPEG`, **deinit camera**, reinit with new pixformat/framesize, capture, then **deinit** and restore JPEG init for streaming.

Important detail: we added a helper `init_camera_with_format(framesize_t, pixformat_t)` for clean reinit.

---

## 7) Capture Endpoint (Master) Details

Path: `/api/capture` (HTTP GET)

Supported query params:
- `session` (string)
- `frame_count` (int)
- `framesize`
- `pixel_format`
- plus any other sensor settings in query string

Flow (master):
1. Parse query
2. Stop stream: `s_stream_enabled = false`
3. Wait until `s_stream_in_progress == false` (up to ~1s)
4. Send POST to slave at `http://slavecam-<id>.local/capture` with same query string
5. Delay `CAPSEQ_SLAVE_PREPARE_DELAY_MS` (default 3000)
6. If capture format != JPEG:
   - `esp_camera_deinit()`
   - `gpio_uninstall_isr_service()`
   - delay 50ms
   - `init_camera_with_format(fs, fmt)`
7. Apply sensor settings from query
8. Drop `CAPSEQ_DROP_FRAMES`
9. Trigger GPIO to slave (GPIO16 default)
10. Capture loop for `frame_count` frames
11. Save file to `/eMMC/capture/<session>-<timestamp>.<ext>` where ext maps to format
12. If reinit was done: deinit + uninstall ISR + delay 50ms + `init_camera()` to restore JPEG

File extensions used:
- JPEG -> `.jpg`
- RGB565 -> `.rgb565`
- RAW -> `.raw`
- GRAYSCALE -> `.gray`
- YUV422 -> `.yuv`

**GPIO ISR conflict fix**: added `gpio_uninstall_isr_service()` before each reinit to avoid `gpio_install_isr_service already installed` warnings.

---

## 8) Streaming Behavior (Master)

- `/stream` uses MJPEG multipart boundary.
- `stream_handler` forces JPEG before loop to avoid mismatch.
- `/api/stream/start` sets `s_stream_enabled = true`
- `/api/stream/stop` sets `s_stream_enabled = false`

**For slave**: streaming might be optional. If you keep streaming, always enforce JPEG.

---

## 9) Sensor Setting Endpoint

`/api/sensor` supports both:
- `application/json`
- URL encoded form

Bounds checking is implemented for the major sensor controls: brightness, contrast, saturation, quality, gain, etc.

This is master code but can be reused for slave if you want remote configuration of slave camera.

---

## 10) Wi‑Fi Configuration

- Configured by `CONFIG_WIFI_SSID` and `CONFIG_WIFI_PASSWORD` from menuconfig (Kconfig).
- `esp_wifi_set_storage(WIFI_STORAGE_RAM)` to avoid flash writes during camera DMA.
- Authmode threshold: `WPA2` if password length > 0, else `OPEN`.

Earlier warning:
`Password length is zero but authmode threshold is 3` — fixed by above logic.

---

## 11) mDNS

Master hostname: `mastercam-<id>.local` via Kconfig `MASTER_ID`.
Slave expected at `slavecam-<id>.local` via Kconfig `SLAVE_ID`.

Observed error:
```
getaddrinfo slavecam-000000.local failed
```
means slave not on network, or mDNS not running, or hostname mismatch.

**Recommendation for slave**: ensure mDNS init and hostname match; or allow master to use static IP config.

---

## 12) Partition Table (4MB)

Custom `partitions.csv`:
```
nvs,       data, nvs,     0x9000,  0x6000
phy_init,  data, phy,     0xF000,  0x1000
factory,   app,  factory, 0x10000, 0x180000
www,       data, spiffs,  0x190000,0x80000
storage,   data, fat,     0x210000,0x1F0000
```

Flash size in `sdkconfig`: 4MB.

For slave, if you don’t need web UI, you can shrink or remove SPIFFS, but keep FAT if you store captures.

---

## 13) Build Dependencies

`main/idf_component.yml` uses component manager to fetch:
- `espressif/mdns`
- `espressif/esp32-camera`

`main/CMakeLists.txt` requires:
- `esp_http_server`
- `esp_http_client`
- `esp_wifi`
- `nvs_flash`
- `mdns`
- `fatfs`
- `spiffs`
- `esp_timer`
- `driver`
- `esp32-camera`
- `json`

Also uses `spiffs_create_partition_image(www ...)` for `main/www`.

---

## 14) Web UI

SPIFFS hosts `main/www/index.html` (Bulma-like theme). Controls:
- Start/stop stream
- Capture sequence form (session, count, framesize, pixel format)
- Full sensor settings (including AWB/AEC, etc.)
- Presets
- Live status via `/api/status`

Slave likely doesn’t need full UI. Consider a stripped-down UI or disable SPIFFS to save flash.

---

## 15) Known Runtime Issues + Fixes Applied

1) **Flash size mismatch**
   - Fixed by switching to 4MB config and smaller partition table.

2) **Camera not supported**
   - Fixed by correct AiThinker pin map (PWDN=32, etc.).

3) **PSRAM allocation failure**
   - Fixed by using DRAM framebuffers and smaller framesize for initial init.

4) **JPEG SOI missing (NO-SOI)**
   - Root cause: setting sensor to RGB565 without reinit.
   - Fix: deinit/reinit camera for non-JPEG capture formats.

5) **Cache/DMA panic when Wi‑Fi writes to NVS**
   - Fixed by Wi‑Fi storage in RAM.

6) **GPIO ISR service already installed**
   - Fixed by calling `gpio_uninstall_isr_service()` before reinit.

---

## 16) How Slave Should Differ From Master

The slave should:
- Expose a `/capture` endpoint that accepts the same parameters (framesize, pixel_format, etc.).
- On `/capture` request, configure camera and **wait for GPIO sync trigger** (from master) before capturing.
- Likely **no stream** needed; or if present, must always use JPEG.
- Save files with session/timestamp using same naming or separate directory (e.g., `/eMMC/capture_slave/`).

Suggested slave flow on `/capture`:
1. Parse request
2. Stop any streaming
3. If pixformat != JPEG -> deinit/reinit with requested format
4. Apply sensor settings
5. Drop frames
6. **Wait on GPIO trigger** (e.g., interrupt or polling on GPIO16 input)
7. Capture N frames, save
8. Restore JPEG init if needed

GPIO trigger should be configured as **input** on slave, since master drives the line.

---

## 17) GPIO Sync

Master:
- GPIO16 is output for trigger
- Pulses high for ~200us before capture

Slave:
- GPIO16 should be configured input, likely with pulldown
- You can implement a simple wait loop on `gpio_get_level()` or use interrupt.

---

## 18) File Formats & Conversion

RGB565 captures saved as `.rgb565`. A helper script `rgb565.py` was created for conversion:

Usage:
```
python rgb565.py file.rgb565 --width W --height H --plot
```

It can output PNG/PPM or plot with numpy/matplotlib.

---

## 19) Important Code Points to Reuse for Slave

- `init_camera()` and `init_camera_with_format()` (same pin map, JPEG default)
- `apply_sensor_settings_from_query()`
- `parse_framesize()` / `parse_pixformat()`
- `send_slave_prepare()` logic (not needed on slave)
- NVS + Wi-Fi init (if slave needs network)
- mDNS init (hostname `slavecam-<id>.local`)

---

## 20) Example Slave Endpoint Contract

**POST** `/capture`
- Body or query must include: `session`, `framesize`, `pixel_format`, other sensor settings
- Slave returns `OK` after capturing

Note: In master, slave endpoint is called at `http://slavecam-<id>.local/capture` with the same query string. Keep this endpoint consistent.

---

## 21) Build / Flash Commands (Master)

All builds were done with:
```
source /home/hamim-mahmud/esp/esp-idf/export.sh
IDF_SKIP_CHECK_SUBMODULES=1 idf.py -p /dev/ttyUSB0 build
IDF_SKIP_CHECK_SUBMODULES=1 idf.py -p /dev/ttyUSB0 flash
```

---

## 22) Additional Suggestions for Slave

- If slave doesn’t need SPIFFS UI, remove `spiffs` component + partition, reduce flash usage.
- Consider using static IP or storing IP in NVS to reduce mDNS reliance.
- Use `esp_camera_deinit()` + reinit to switch formats, identical to master capture fix.
- Keep JPEG for streaming if you choose to stream.

---

## 23) Known Good Logs

A good capture run after reinit changes shows:
```
cam_hal: cam init ok
camera: Detected OV3660 camera
cam_hal: cam config ok
mastercam: path: /eMMC/capture/session1-xxxx.rgb565
...
```

Warnings about GPIO ISR were fixed by uninstalling ISR between reinit cycles.

---

## 24) Summary of Key Slave Requirements

- Use same pin map and camera config helper
- Implement `/capture` endpoint that can be called by master
- Wait for GPIO trigger for synchronized capture
- Support pixformat reinit for non-JPEG
- Use 4MB flash config and partition table

---

End of summary.
