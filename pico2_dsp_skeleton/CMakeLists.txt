cmake_minimum_required(VERSION 3.13)

# Default to Pico 2 (rp2350 platform) if not specified via command line
if(NOT PICO_BOARD)
    set(PICO_BOARD pico2 CACHE STRING "Board type")
endif()
if(NOT PICO_PLATFORM)
    set(PICO_PLATFORM rp2350 CACHE STRING "Target hardware platform")
endif()

include(pico_sdk_import.cmake)

project(pico2_dsp_skeleton C CXX ASM)

pico_sdk_init()

add_executable(pico2_dsp_skeleton
    main.c
)

pico_generate_pio_header(pico2_dsp_skeleton ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)

set(CMSISCORE ${PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Core CACHE PATH "CMSIS Core Include Path")
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../CMSIS-DSP CMSIS-DSP)

target_link_libraries(pico2_dsp_skeleton
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_clocks
    CMSISDSP
)

pico_add_extra_outputs(pico2_dsp_skeleton)

# Enable USB stdio, disable UART stdio
pico_enable_stdio_usb(pico2_dsp_skeleton 1)
pico_enable_stdio_uart(pico2_dsp_skeleton 0)
