# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/hamim-mahmud/esp/esp-idf/components/bootloader/subproject"
  "/home/hamim-mahmud/esp/esp-idf/hamim/new_from_codex/build/bootloader"
  "/home/hamim-mahmud/esp/esp-idf/hamim/new_from_codex/build/bootloader-prefix"
  "/home/hamim-mahmud/esp/esp-idf/hamim/new_from_codex/build/bootloader-prefix/tmp"
  "/home/hamim-mahmud/esp/esp-idf/hamim/new_from_codex/build/bootloader-prefix/src/bootloader-stamp"
  "/home/hamim-mahmud/esp/esp-idf/hamim/new_from_codex/build/bootloader-prefix/src"
  "/home/hamim-mahmud/esp/esp-idf/hamim/new_from_codex/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/hamim-mahmud/esp/esp-idf/hamim/new_from_codex/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/hamim-mahmud/esp/esp-idf/hamim/new_from_codex/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
