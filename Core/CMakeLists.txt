set(PROJECT stm32)

file(GLOB BSP_SOURCES 
	${CMAKE_SOURCE_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/*.c
)
set(BSP_SOURCES ${BSP_SOURCES})
add_library (${PROJECT} OBJECT ${BSP_SOURCES})
target_include_directories (${PROJECT} PUBLIC 
	${CMAKE_SOURCE_DIR}/Drivers/STM32F3xx_HAL_Driver/Inc
	${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F3xx/Include
	${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
	${CMAKE_CURRENT_SOURCE_DIR}/Inc
	)

set(MAIN_SOURCES
    Src/main.c
    Src/crc.cpp
    Src/stm32f3xx_hal_msp.c
    Src/stm32f3xx_it.c
    Src/system_stm32f3xx.c
	# STARTUP SCRIPT
    Startup/startup_stm32f302r8tx.s
	)
set(EXECUTABLE ${PROJECT_NAME}.elf)
add_executable(${EXECUTABLE} ${MAIN_SOURCES})
target_link_libraries (${EXECUTABLE} LINK_PUBLIC stm32 application) 

# Print executable size
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND ${SIZE_CMD} ${EXECUTABLE}
)
# Create hex file
set (BIN_PATH ${CMAKE_SOURCE_DIR}/build)
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND cp ${EXECUTABLE} ${BIN_PATH}/
        COMMAND ${OBJCOPY_CMD} -O ihex   ${BIN_PATH}/${EXECUTABLE} ${BIN_PATH}/${PROJECT_NAME}.hex
        COMMAND ${OBJCOPY_CMD} -O binary ${BIN_PATH}/${EXECUTABLE} ${BIN_PATH}/${PROJECT_NAME}.bin
)
