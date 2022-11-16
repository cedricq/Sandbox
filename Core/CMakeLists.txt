set(PROJECT stm32)

if(NOT ${TEST})

file(GLOB STM32_SOURCES 
	${CMAKE_SOURCE_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/*.c
)
set(STM32_SOURCES ${STM32_SOURCES})
add_library (${PROJECT}_driver ${STM32_SOURCES})
target_include_directories (${PROJECT}_driver PUBLIC 
	${CMAKE_SOURCE_DIR}/Drivers/STM32F3xx_HAL_Driver/Inc
	${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F3xx/Include
	${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
	Inc
	)
target_compile_definitions(${PROJECT}_driver PRIVATE
    ${MCU_MODEL}
    USE_HAL_DRIVER
    DEBUG 
    )
set(MAIN_GENERATED_FILES
    Src/main.c
    Src/crc.cpp
    Src/stm32f3xx_hal_msp.c
    Src/stm32f3xx_it.c
    Src/system_stm32f3xx.c
	# STARTUP SCRIPT
    Startup/startup_stm32f302r8tx.s
	)
set(EXECUTABLE ${PROJECT_NAME}.elf)
add_executable(${EXECUTABLE} ${MAIN_GENERATED_FILES})
target_link_libraries (${EXECUTABLE} LINK_PUBLIC stm32_driver application) 

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

endif()
