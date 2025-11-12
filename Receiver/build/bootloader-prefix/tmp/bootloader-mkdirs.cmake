# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/yrs/esp/v5.5/esp-idf/components/bootloader/subproject"
  "/home/yrs/Documents/esp32/v5.5/AlarmLora/Receiver/build/bootloader"
  "/home/yrs/Documents/esp32/v5.5/AlarmLora/Receiver/build/bootloader-prefix"
  "/home/yrs/Documents/esp32/v5.5/AlarmLora/Receiver/build/bootloader-prefix/tmp"
  "/home/yrs/Documents/esp32/v5.5/AlarmLora/Receiver/build/bootloader-prefix/src/bootloader-stamp"
  "/home/yrs/Documents/esp32/v5.5/AlarmLora/Receiver/build/bootloader-prefix/src"
  "/home/yrs/Documents/esp32/v5.5/AlarmLora/Receiver/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/yrs/Documents/esp32/v5.5/AlarmLora/Receiver/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/yrs/Documents/esp32/v5.5/AlarmLora/Receiver/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
