# ESP32-SHITL-Tx: network-controlled, timed 2.4 GHz transmitter (esp32c6 / esp32s3).
cmake_minimum_required(VERSION 3.16)
# Reported as ver= in the STATUS line; bump it when shipping an OTA image.
set(PROJECT_VER "0.1.1")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(shitl_tx)
