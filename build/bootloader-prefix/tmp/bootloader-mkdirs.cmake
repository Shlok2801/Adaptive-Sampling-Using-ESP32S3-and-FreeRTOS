# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/ESP/v5.2.1/esp-idf/components/bootloader/subproject"
  "D:/ESP/v5.2.1/esp-idf/projects/esp-filter/Adaptive-Sampling-Using-ESP32S3-and-FreeRTOS/build/bootloader"
  "D:/ESP/v5.2.1/esp-idf/projects/esp-filter/Adaptive-Sampling-Using-ESP32S3-and-FreeRTOS/build/bootloader-prefix"
  "D:/ESP/v5.2.1/esp-idf/projects/esp-filter/Adaptive-Sampling-Using-ESP32S3-and-FreeRTOS/build/bootloader-prefix/tmp"
  "D:/ESP/v5.2.1/esp-idf/projects/esp-filter/Adaptive-Sampling-Using-ESP32S3-and-FreeRTOS/build/bootloader-prefix/src/bootloader-stamp"
  "D:/ESP/v5.2.1/esp-idf/projects/esp-filter/Adaptive-Sampling-Using-ESP32S3-and-FreeRTOS/build/bootloader-prefix/src"
  "D:/ESP/v5.2.1/esp-idf/projects/esp-filter/Adaptive-Sampling-Using-ESP32S3-and-FreeRTOS/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/ESP/v5.2.1/esp-idf/projects/esp-filter/Adaptive-Sampling-Using-ESP32S3-and-FreeRTOS/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/ESP/v5.2.1/esp-idf/projects/esp-filter/Adaptive-Sampling-Using-ESP32S3-and-FreeRTOS/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
