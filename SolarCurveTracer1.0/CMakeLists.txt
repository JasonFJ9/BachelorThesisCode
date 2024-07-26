# Specify required cmake version
cmake_minimum_required(VERSION 3.20)

# Include the pico sdk
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Define the project
project(SolarCurveTracer C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the pico sdk
pico_sdk_init()

# turn on all compiler warnings 
add_compile_options(-Wall)

# Add the source files
add_executable(${PROJECT_NAME}
  main.cpp

)

add_subdirectory(lib/no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI build)

# Generate the needed output files
pico_add_extra_outputs(${PROJECT_NAME})

add_library(mcp23017lib INTERFACE)
add_library(tmp1075lib INTERFACE)
add_library(veml7700lib INTERFACE)

add_library(pac1934lib INTERFACE)
add_library(hal_gpio INTERFACE)
add_library(hal_i2c INTERFACE)
add_library(hal_sleep INTERFACE)
add_library(hal_spi INTERFACE)
add_library(hal_uart  INTERFACE)


target_sources(mcp23017lib INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/src/mcp23017/mcp23017.c
)
target_sources(tmp1075lib INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/src/tmp1075/tmp1075.cpp
)
target_sources(veml7700lib INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/src/VEML7700/VEML7700.cpp
)

target_sources(pac1934lib INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/src/Pac1934/Pac193x.c
)
target_sources(hal_gpio INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/hal/gpio/Gpio.c
)
target_sources(hal_i2c INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/hal/i2c/I2c.c
)
target_sources(hal_sleep INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/hal/sleep/Sleep.c
)
target_sources(hal_spi INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/hal/spi/Spi.c
)
target_sources(hal_uart INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/hal/uart/Uart.c
)


target_include_directories(mcp23017lib INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(tmp1075lib INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(veml7700lib INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

target_include_directories(pac1934lib INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(hal_gpio INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(hal_i2c INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(hal_sleep INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(hal_spi INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(hal_uart INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)


# Link the needed libraries
target_link_libraries(${PROJECT_NAME}
  pico_stdlib
  hardware_i2c
  mcp23017lib
  tmp1075lib
  veml7700lib
  pico_bootrom pico_stdio_usb hardware_spi pac1934lib hal_gpio hal_i2c hal_sleep hal_spi hal_uart
  FatFs_SPI
)



# Use USB for printf
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)


