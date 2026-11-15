# Top-level project file for the BEESDR ESP32-Tx WiFi TX beacon (esp32c6 / esp32s3).
# Build with the ESP-IDF (v5.1+):  idf.py set-target esp32c6|esp32s3 && idf.py build
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(beesdr_wifi_tx)
