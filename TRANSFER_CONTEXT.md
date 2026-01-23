# Transfer Context (ESP32-CAM PSRAM Debugging)

Date: 2026-01-23
Project: /home/hamim-mahmud/esp/esp-idf/hamim/new_from_codex
Board: AI-Thinker ESP32-CAM (ESP32-D0WD-V3, PSRAM 8MB, Flash 4MB)
IDF: v5.5.2-5-gc71a3906e4-dirty

## Goal
Use PSRAM only for camera frame buffers (explicit MALLOC_CAP_SPIRAM). Keep Wi-Fi/SD/FATFS/general malloc in internal RAM.

## Symptoms
- With PSRAM added to heap: heap corruption / LoadProhibited panics in TLSF or VFS.
- With PSRAM disabled: SD mount + Wi-Fi stable.
- Current crash: first PSRAM allocation for camera frame buffer.

## Latest Boot Evidence
- PSRAM heap add succeeds ("Adding pool of 4096K of PSRAM memory to heap allocator")
- Log shows free SPIRAM bytes, then panic during cam_dma_config frame buffer allocation.

## Current Strategy
- Disable PSRAM boot init; manual PSRAM init right before camera init.
- Use CAPS_ALLOC (explicit MALLOC_CAP_SPIRAM only).
- Patch ESP-IDF to remove MALLOC_CAP_DEFAULT from PSRAM heap when not in USE_MALLOC mode.
- SD/Wi-Fi/FATFS forced internal.
- Bank switching disabled, 2T mode enabled.
- PSRAM heap test after add (esp_psram_extram_test).

## Files Modified
- main/app_main.c
  - Force frame buffers to PSRAM
  - Add init_psram_for_camera() with one-time heap add + log + extram_test
  - Set logging to ROM vprintf
  - Move mount_sdcard() before Wi-Fi init
- sdkconfig
  - Safe PSRAM config (CAPS_ALLOC, BOOT_INIT off, Wi-Fi/LWIP in internal, FATFS internal)
  - 40MHz PSRAM, cache workaround, 2T mode, no bank switching
- esp-idf/components/esp_psram/system_layer/esp_psram.c
  - Patch: only include MALLOC_CAP_DEFAULT in PSRAM heap when CONFIG_SPIRAM_USE_MALLOC is enabled

## Current sdkconfig PSRAM Section (snapshot)
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_QUAD=y
CONFIG_SPIRAM_TYPE_AUTO=y
CONFIG_SPIRAM_SPEED_40M=y
CONFIG_SPIRAM_BOOT_HW_INIT=y
# CONFIG_SPIRAM_BOOT_INIT is not set
CONFIG_SPIRAM_PRE_CONFIGURE_MEMORY_PROTECTION=y
# CONFIG_SPIRAM_USE_MEMMAP is not set
CONFIG_SPIRAM_USE_CAPS_ALLOC=y
# CONFIG_SPIRAM_USE_MALLOC is not set
# CONFIG_SPIRAM_MEMTEST is not set
# CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP is not set
CONFIG_SPIRAM_CACHE_WORKAROUND=y
CONFIG_SPIRAM_CACHE_WORKAROUND_STRATEGY_MEMW=y
# CONFIG_SPIRAM_BANKSWITCH_ENABLE is not set
CONFIG_SPIRAM_2T_MODE=y
CONFIG_SPIRAM_OCCUPY_NO_HOST=y

## App-side PSRAM init (app_main.c)
- init_psram_for_camera():
  - if not initialized: esp_psram_init()
  - add to heap once: esp_psram_extram_add_to_heap_allocator()
  - log free bytes: heap_caps_get_free_size(MALLOC_CAP_SPIRAM)
  - run esp_psram_extram_test()

## Last Known Crash
After PSRAM heap added and camera init starts:
- cam_hal: Allocating frame buffer in PSRAM
- Panic in TLSF / spinlock during heap allocation

## Next Debug Ideas
- If extram_test fails: PSRAM unstable; try lower frame size, check power, or disable PSRAM for camera.
- If extram_test passes but allocation still crashes: PSRAM mapping/allocator mismatch; consider MEMMAP-only + custom allocator or fully internal camera buffers.

