# The portable core is compiled straight from firmware/src: the turbine and
# the Python software-in-the-loop harness run the same sources.
set(KS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)
file(GLOB KS_CORE_SRCS ${KS_ROOT}/src/*.c)

idf_component_register(
    SRCS main.c hal_radio.c hal_sensors.c hal_yaw.c hal_store.c hal_log.c console.c ${KS_CORE_SRCS}
    INCLUDE_DIRS . ${KS_ROOT}/include
    PRIV_INCLUDE_DIRS ${KS_ROOT}/src
    REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_adc esp_timer
             esp_driver_gpio esp_driver_ledc esp_driver_i2c esp_driver_uart)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Wdouble-promotion -Werror)
