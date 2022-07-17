cmake_minimum_required(VERSION 3.6)

set(CMAKE_TOOLCHAIN_FILE arm_toolchain.cmake)

set(SOURCE_FILES "")

project(sandbox LANGUAGES C CXX ASM)

#add_subdirectory(xmclib)

file(GLOB Drivers_SRC
     Drivers/STM32F3xx_HAL_Driver/Src/*.c
     Drivers/STM32F3xx_HAL_Driver/Inc/*.h
     Drivers/STM32F3xx_HAL_Driver/Inc/Legacy/*.h
)

file(GLOB Application_SRC
     Application/src/*
     Application/inc/*
)

file(GLOB Arch_SRC
     Arch/src/*
     Arch/inc/*
)

include(CMakePrintHelpers)
cmake_print_variables(Drivers_SRC)

#message(STATUS "Drivers_SRC: ")

#message(STATUS "Application_SRC: ${Application_SRC}")

add_executable(
    ${PROJECT_NAME}.elf
    
    ${Drivers_SRC}
    ${Application_SRC}
    ${Arch_SRC}
    
    Core/Src/stm32f3xx_hal_msp.c
    Core/Src/stm32f3xx_it.c
    Core/Src/system_stm32f3xx.c
    Core/Src/main.c
    Core/Src/crc.cpp
    Core/Startup/startup_stm32f302r8tx.s
    
)

#target_link_libraries(${PROJECT_NAME}.elf xmclib)
include_directories(
	Core/Inc
	Drivers/CMSIS/Device/ST/STM32F3xx/Include
	Drivers/CMSIS/Include
	Drivers/STM32F3xx_HAL_Driver/Inc
	Drivers/STM32F3xx_HAL_Driver/Inc/Legacy
	Application/inc
	Arch/inc
)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
   COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
)

