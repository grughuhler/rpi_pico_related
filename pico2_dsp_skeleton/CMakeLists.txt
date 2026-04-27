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

set(CMSISCORE ${PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Core CACHE PATH "CMSIS Core Include Path")
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../CMSIS-DSP CMSIS-DSP)

# Create a shared static library for the common framework code
add_library(dsp_common STATIC main.c dsp_common.c)

# Generate PIO header once for the common library
pico_generate_pio_header(dsp_common ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)

target_link_libraries(dsp_common PUBLIC
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_clocks
    CMSISDSP
)

# Enable maximum math / loop optimizations for common
target_compile_options(dsp_common PUBLIC -Ofast)

# List of all DSP modules
set(DSP_MODULES fir iir make_sine mult detect fft fft_filter none)

# Create an executable target for each module
foreach(module IN LISTS DSP_MODULES)
    add_executable(${module} dsp_${module}.c)
    
    target_link_libraries(${module} PRIVATE dsp_common)
    
    pico_add_extra_outputs(${module})
    
    # Enable USB stdio, disable UART stdio
    pico_enable_stdio_usb(${module} 1)
    pico_enable_stdio_uart(${module} 0)
endforeach()
