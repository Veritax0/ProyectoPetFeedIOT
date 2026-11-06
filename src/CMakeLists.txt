# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
    esp_wifi
    esp_event
    esp_netif
    nvs_flash
    esp_driver_gpio
    espressif__mqtt
)
