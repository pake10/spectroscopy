cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(ilx554a_pico C CXX ASM)
pico_sdk_init()

add_executable(ilx554a_pico main.c)
pico_generate_pio_header(ilx554a_pico ${CMAKE_CURRENT_LIST_DIR}/ilx554a.pio)

target_link_libraries(ilx554a_pico
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_adc
)

pico_enable_stdio_usb(ilx554a_pico 1)
pico_enable_stdio_uart(ilx554a_pico 0)
pico_add_extra_outputs(ilx554a_pico)