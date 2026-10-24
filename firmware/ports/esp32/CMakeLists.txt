# ESP-IDF project for the reference node (ESP32-WROOM-32E).
#   . $IDF_PATH/export.sh && idf.py set-target esp32 && idf.py build flash monitor
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(kalonda_node)
