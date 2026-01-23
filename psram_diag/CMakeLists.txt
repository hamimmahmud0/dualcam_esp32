cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    /home/hamim-mahmud/esp/esp-idf/examples/esp32cam/esp32-cam-websrv/managed_components/espressif__esp32-camera
    /home/hamim-mahmud/esp/esp-idf/examples/esp32cam/esp32-cam-websrv/managed_components/espressif__esp_jpeg
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(psram_diag)
