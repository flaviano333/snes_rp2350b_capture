cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD spotpear_rp2350b_mini_a CACHE STRING "RP2350B board")
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards CACHE STRING "Custom board headers")

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(snes_rp2350b_romsel_qualified C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(snes_rp2350b_romsel_qualified main.c)

pico_generate_pio_header(snes_rp2350b_romsel_qualified ${CMAKE_CURRENT_LIST_DIR}/capture_timing_sweep.pio)
pico_generate_pio_header(snes_rp2350b_romsel_qualified ${CMAKE_CURRENT_LIST_DIR}/capture_read_trigger.pio)

target_link_libraries(snes_rp2350b_romsel_qualified
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_gpio
    hardware_clocks
)

pico_enable_stdio_usb(snes_rp2350b_romsel_qualified 1)
pico_enable_stdio_uart(snes_rp2350b_romsel_qualified 0)

target_compile_definitions(snes_rp2350b_romsel_qualified PRIVATE
    PICO_PIO_USE_GPIO_BASE=1
)

pico_add_extra_outputs(snes_rp2350b_romsel_qualified)
