

cmake_minimum_required(VERSION 3.16)

# Pull in PICO SDK (must be before project)
include(pico_sdk_import.cmake)


project(st7735 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()


add_executable(st7735
        main.cpp
         spi.cpp
        ./ST7735_pico/ST7735_pico.cpp
        ./ST7735_pico/glcdfont.c
        ./ST7735_pico/ST7735_font_ComicSansMS.c
        ./ST7735_pico/ST7735_font_Arial.c
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(st7735
        pico_stdlib 
        hardware_i2c
        hardware_spi
        )

# enable usb output, disable uart output
pico_enable_stdio_usb(st7735 1)
pico_enable_stdio_uart(st7735 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(st7735)

