# MasterCam: ESP32-CAM Dual Sync + Capture-Only

MasterCam is a dual ESP32-CAM capture system with three firmware roles:
- Master: web UI, MJPEG streaming, and synchronized capture control.
- Slave: responds to sync commands and captures in lock-step with the master.
- Capture-only: no Wi-Fi or web server, just local capture to SD card.

The master and slave coordinate capture timing over UDP to align frames across devices.
Captured files are written to the SD card under `/eMMC/capture` with timestamped names.

## Features
- Dual-camera synchronized capture using UDP time sync
- MJPEG streaming (port 81) with start/stop control
- Web UI hosted from SPIFFS (`main/www/index.html`)
- Sensor tuning via HTTP API (JSON or form)
- Flexible capture parameters (framesize, pixel format, frame count)
- Capture-only mode for offline logging

## Hardware
Designed around the AI-Thinker ESP32-CAM pin map:

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

Other boards may require pin edits in:
- `main/app_main.c`
- `main/app_main_slave.c`
- `main/app_main_capture_only.c`

## Project layout
```
.
├── main/
│   ├── app_main.c              Master firmware
│   ├── app_main_slave.c        Slave firmware
│   ├── app_main_capture_only.c Capture-only firmware
│   ├── Kconfig.projbuild       Menuconfig options
│   └── www/index.html          UI served from SPIFFS
├── rgb565.py                   Convert RGB565 frames to PNG/PPM
├── partitions.csv              Includes SPIFFS partition for UI
└── README.md
```

## Build and flash
1) Set the target and open menuconfig:
```
idf.py set-target esp32
idf.py menuconfig
```

2) Configure the role and IDs (Menu: `MasterCam`):
- Master (default): no special flag
- Slave: enable `APP_ROLE_SLAVE`
- Capture-only: enable `APP_ROLE_CAPTURE_ONLY`

3) Build and flash:
```
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Configuration (menuconfig: MasterCam)
Key options in `main/Kconfig.projbuild`:
- `MASTER_ID`, `SLAVE_ID`: used for mDNS hostnames
  - Master hostname: `mastercam-<MASTER_ID>.local`
  - Slave hostname: `slavecam-<SLAVE_ID>.local`
- `WIFI_SSID`, `WIFI_PASSWORD`: Wi-Fi credentials
- `CAPSEQ_*`: capture sync timing, UDP port, retries, and safety margins
- `CAPSEQ_ALLOW_SLAVE_MISSING`: allow master capture without slave

## Running
- Master UI: `http://mastercam-<MASTER_ID>.local/`
- Slave UI: `http://slavecam-<SLAVE_ID>.local/`
- MJPEG stream: `http://<host>:81/stream`

If mDNS does not resolve, use the IP printed in `idf.py monitor` logs.

## Capture storage
Captured files go to `/eMMC/capture` and are named:
```
<session>-<timestamp_ms>.<ext>
```
Extensions are based on pixel format:
- JPEG: `.jpg`
- RGB565: `.rgb565`
- Grayscale: `.gray`
- YUV422: `.yuv`

Important:
- Master/Slave firmware mounts SD without formatting.
- Capture-only firmware formats the SD card on every boot.

## API documentation

### Common endpoints (master and slave)

`GET /`
- Serves the SPIFFS UI (or a built-in fallback page).

`GET /api/status`
- Returns JSON status.
- Master fields: `stream_enabled`, `stream_active`, `uptime_ms`, `free_heap`, `slave_id`, `master_id`.
- Slave fields: `stream_enabled`, `stream_active`, `capture_ready`, `capture_active`, `uptime_ms`, `free_heap`, `slave_id`.

`GET /api/stream/start`
- Enables MJPEG streaming and returns `OK`.

`GET /api/stream/stop`
- Disables MJPEG streaming and returns `OK`.

`GET /stream` (port 81)
- MJPEG multipart stream.
- Returns 409 if streaming is disabled or already active.

`POST /api/sensor`
- Updates sensor settings.
- Accepts either:
  - `application/x-www-form-urlencoded`
  - `application/json`
- Returns `OK`.

Supported sensor keys (ranges are clamped):
- `framesize`: qqvga, qvga, vga, svga, xga, sxga, uxga (or numeric)
- `pixel_format`: jpeg, rgb565, grayscale, yuv422 (or numeric)
- `quality` (2..63)
- `brightness` (-2..2)
- `contrast` (-2..2)
- `saturation` (-2..2)
- `gainceiling` (0..6)
- `colorbar` (0/1)
- `awb` (0/1)
- `awb_gain` (0/1)
- `wb_mode` (0..4)
- `aec2` (0/1)
- `ae_level` (-2..2)
- `aec_value` (0..1200)
- `agc` (0/1)
- `agc_gain` (0..30)
- `gain_ctrl` (0/1)
- `bpc` (0/1)
- `wpc` (0/1)
- `raw_gma` (0/1)
- `lenc` (0/1)
- `hmirror` (0/1)
- `vflip` (0/1)
- `dcw` (0/1)
- `special_effect` (0..6)
- `exposure_ctrl` (0/1)

### Capture endpoints

Master capture:
`GET /api/capture`
- Blocks until capture completes.
- Query parameters:
  - `session` (string, default: `session`)
  - `frame_count` (int, default: 1)
  - `framesize` (see above)
  - `pixel_format` (see above)
  - `cpu_time_to_start` (ms; overrides `CAPSEQ_SYNC_SAFETY_MS`)
  - Any sensor keys listed above

Slave capture (prepare only):
- `GET /api/capture`
- `POST /api/capture`
- `GET /capture` (legacy)
- `POST /capture` (legacy)

The slave returns `OK` after it prepares the camera. The actual capture start
is triggered by the master via UDP `START`.

### UDP sync protocol (master <-> slave)
UDP port: `CONFIG_CAPSEQ_SYNC_UDP_PORT` (default 65)

Messages:
- `READY` -> slave replies `OK` when ready
- `<master_time_us>` -> slave replies with `<slave_time_us>`
- `START <delay_us>` -> slave replies `ACK` and starts capture after delay

The master uses these to estimate round-trip time and CPU clock disparity, then
schedules both cameras to start at aligned timestamps.

## Usage examples
Set a host name once:
```
MASTER_HOST=mastercam-000001.local
SLAVE_HOST=slavecam-000002.local
```

Status:
```
curl "http://$MASTER_HOST/api/status"
```

Start and stop streaming:
```
curl "http://$MASTER_HOST/api/stream/start"
curl "http://$MASTER_HOST/api/stream/stop"
```

Capture 10 JPEG frames:
```
curl "http://$MASTER_HOST/api/capture?session=test&frame_count=10&framesize=svga&pixel_format=jpeg"
```

Set sensor parameters (form):
```
curl -X POST "http://$MASTER_HOST/api/sensor" \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -d "framesize=svga&quality=10&brightness=1"
```

Set sensor parameters (JSON):
```
curl -X POST "http://$MASTER_HOST/api/sensor" \
  -H "Content-Type: application/json" \
  -d '{"framesize":"svga","quality":10,"brightness":1}'
```

## Working with RGB565 captures
Use `rgb565.py` to convert raw RGB565 frames into PNG or PPM.

Common frame sizes:
- QQVGA: 160x120
- QVGA: 320x240
- VGA: 640x480
- SVGA: 800x600
- XGA: 1024x768
- SXGA: 1280x1024
- UXGA: 1600x1200

Example:
```
python3 rgb565.py /path/to/frame.rgb565 --width 640 --height 480 --out frame.png
```

## Capture-only firmware
Capture-only mode is in `main/app_main_capture_only.c`.
It formats the SD card, captures a fixed sequence, and stops.
Key constants:
- `CAPTURE_FRAME_COUNT`
- `CAPTURE_DROP_FRAMES`
- `CAPTURE_SESSION`

Adjust these in `main/app_main_capture_only.c` to change behavior.

## Troubleshooting
- `409 stream disabled`: call `/api/stream/start` before `/stream`.
- `409 capture busy`: another capture is already in progress.
- No UI: confirm SPIFFS partition `www` exists in `partitions.csv` and the image
  is built (`spiffs_create_partition_image` in `main/CMakeLists.txt`).
- SD mount errors: check wiring and card formatting. Master/slave do not auto-format.
- Capture-only wipes the card each boot (by design).

## Notes
- Master and slave default to JPEG for streaming stability.
- Non-JPEG capture modes trigger a camera re-init to reconfigure the DMA path.
- If you see PSRAM or DMA issues, review `TRANSFER_CONTEXT.md` for debugging notes.
