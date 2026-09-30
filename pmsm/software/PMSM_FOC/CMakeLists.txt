cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME PMSM_FOC)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/Board/Pwm/Pwm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Board/Watchdog/watchdog.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Board/Dio/Dio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Board/Adc/Board_Adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Board/Uart/Board_Uart1.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Board/Uart/Board_485.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Board/Dac/Board_Dac.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Sensor/Sensor_Tamagawa.c
    ${CMAKE_CURRENT_SOURCE_DIR}/System/System.c
    ${CMAKE_CURRENT_SOURCE_DIR}/System/SystemTime.c
    ${CMAKE_CURRENT_SOURCE_DIR}/System/TaskManager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/System/TaskConfig.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Motor/Motor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Motor/Motor_VF.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Motor/Motor_IF.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Motor/Motor_Speed.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Motor/Motor_Position.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Motor/Motor_Identification.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Foc/FocAlgorithm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Foc/FocTrigTable.c
    ${CMAKE_CURRENT_SOURCE_DIR}/APP/Key.c
    ${CMAKE_CURRENT_SOURCE_DIR}/APP/Led.c
    ${CMAKE_CURRENT_SOURCE_DIR}/APP/MotorApp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/APP/Com.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Protocol/Fifo.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Protocol/Justfloat.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Protocol/Protocol.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/Board/Pwm/
    ${CMAKE_CURRENT_SOURCE_DIR}/Board/Core/
    ${CMAKE_CURRENT_SOURCE_DIR}/Board/Watchdog/
    ${CMAKE_CURRENT_SOURCE_DIR}/Board/Dio/
    ${CMAKE_CURRENT_SOURCE_DIR}/Board/Adc/
    ${CMAKE_CURRENT_SOURCE_DIR}/Board/Uart/
    ${CMAKE_CURRENT_SOURCE_DIR}/Board/Dac/
    ${CMAKE_CURRENT_SOURCE_DIR}/System/
    ${CMAKE_CURRENT_SOURCE_DIR}/Motor/
    ${CMAKE_CURRENT_SOURCE_DIR}/Foc/
    ${CMAKE_CURRENT_SOURCE_DIR}/APP/
    ${CMAKE_CURRENT_SOURCE_DIR}/Protocol/
    ${CMAKE_CURRENT_SOURCE_DIR}/Sensor/
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m

    # Add user defined libraries
)
