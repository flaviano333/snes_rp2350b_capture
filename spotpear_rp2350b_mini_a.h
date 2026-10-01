#ifndef _BOARDS_SPOTPEAR_RP2350B_MINI_A_H
#define _BOARDS_SPOTPEAR_RP2350B_MINI_A_H

pico_board_cmake_set(PICO_PLATFORM, rp2350)

#define SPOTPEAR_RP2350B_MINI_A

// RP2350B is the 48-GPIO package. In Pico SDK this is selected with PICO_RP2350A=0.
#define PICO_RP2350A 0

// SpotPear advertises 16 Mbyte external QSPI flash on this board.
#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1
#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif
pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (16 * 1024 * 1024))
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif

// On-board WS2812B. We do not drive it in this diagnostic firmware.
#ifndef PICO_DEFAULT_WS2812_PIN
#define PICO_DEFAULT_WS2812_PIN 20
#endif

#endif
