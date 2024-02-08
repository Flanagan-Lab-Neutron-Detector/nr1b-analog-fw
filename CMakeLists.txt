cmake_minimum_required(VERSION 3.12)

# Pull in SDK from git
set(PICO_SDK_FETCH_FROM_GIT on)
include(pico_sdk_import.cmake)

project(nr1b-analog-fw C CXX ASM)

# initialize SDK
pico_sdk_init()

# Rest of the project

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
	message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

add_executable(nr1b-analog-fw
        src/main.c
        src/pindefs.h
)

add_compile_options(-Wall -Wextra)

# Pull in basic dependencies
target_link_libraries(nr1b-analog-fw pico_stdlib hardware_spi hardware_adc)
# SDK options
pico_enable_stdio_usb(nr1b-analog-fw 1)
pico_enable_stdio_uart(nr1b-analog-fw 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(nr1b-analog-fw)

# set program url
pico_set_program_url(nr1b-analog-fw "https://github.com/Flanagan-Lab-Neutron-Detector/nr1b-analog-fw/tree/HEAD")
