cmake_minimum_required(VERSION 3.22)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    message(STATUS "Found CCACHE!")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
endif()

set(CMAKE_C_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(PROJECT_NAME Signal-visualizer-and-generator)

# CMAKE project name, languages and version
project(${PROJECT_NAME}
    LANGUAGES C ASM
    VERSION 0.1
)

# Reduce binary size. Might be used to code to fit in the MCU
option(GARBAGE_COLLECT_SECTIONS "Use -f{function,data}-sections and -Wl,--gc-sections." TRUE)
if (${GARBAGE_COLLECT_SECTIONS})
    add_compile_options(-ffunction-sections -fdata-sections)
    add_link_options(-Wl,--gc-sections)
endif()

set(EXE_NAME ${PROJECT_NAME}.elf)
set(OPENOCD_CFG_NAME ${PROJECT_NAME}.cfg)

add_executable(${EXE_NAME})

FILE(GLOB STM_HAL_DRIVERS Drivers/STM32F1xx_HAL_Driver/Src/*.c)
FILE(GLOB MAIN_FILES **/*.c)

target_sources(${EXE_NAME} PRIVATE
    startup/startup_stm32f103xb.s
    
    "${STM_HAL_DRIVERS}"
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c

    "${MAIN_FILES}"

    Src/application/controller.c
    Src/application/generator.c
    Src/application/timer_handler.c
    Src/application/visualizer.c
)


target_include_directories(${EXE_NAME} PRIVATE
    Drivers/STM32F1xx_HAL_Driver/Inc/
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc/
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc/
    Drivers/CMSIS/Device/ST/STM32F1xx/Include/
    Drivers/CMSIS/Include/
    Inc/
    Drivers/STM32F1xx_HAL_Driver/Inc/
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc/
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc/
    Drivers/CMSIS/Device/ST/STM32F1xx/Include/
    Drivers/CMSIS/Include/
    Inc/
)

target_compile_options(${EXE_NAME} PRIVATE
    -Wall
    -Wextra
    -Wshadow
    -Wno-unknown-pragmas
)

target_link_options(${EXE_NAME} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld
    -Wl,--gc-sections,--no-warn-rwx-segment
    -specs=nano.specs
)

target_compile_definitions(${EXE_NAME} PUBLIC
    USE_HAL_DRIVER
    STM32F103xB
)

add_custom_target(flash
    openocd -f ${CMAKE_CURRENT_SOURCE_DIR}/${OPENOCD_CFG_NAME} -c "program ${EXE_NAME} reset exit"
    VERBATIM
)

add_custom_target(flash_no_reset
    openocd -f ${CMAKE_CURRENT_SOURCE_DIR}/${OPENOCD_CFG_NAME} -c "program ${EXE_NAME} exit"
    VERBATIM
)

add_custom_target(connect
    openocd -f ${CMAKE_CURRENT_SOURCE_DIR}/${OPENOCD_CFG_NAME}
    VERBATIM
)

add_custom_target(debug
    arm-none-eabi-gdb -ex "target extended-remote localhost:3333" ${CMAKE_BINARY_DIR}/${EXE_NAME}
    VERBATIM
)

add_dependencies(flash ${EXE_NAME})
add_dependencies(flash_no_reset ${EXE_NAME})
add_dependencies(debug ${EXE_NAME})

