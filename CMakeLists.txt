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
set(EXE_NAME ${PROJECT_NAME}.elf)
set(OPENOCD_CFG_NAME ${PROJECT_NAME}.cfg)

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


add_library(ST STATIC)

set(HAL_DRIVERS_SOURCES
    Drivers/STM32F1xx_HAL_Driver/Src
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c

    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
)



target_sources(ST PRIVATE
    "${HAL_DRIVERS_SOURCES}"
)

target_include_directories(ST PUBLIC
    Drivers/STM32F1xx_HAL_Driver/Inc/
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc/
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc/
    Drivers/CMSIS/Device/ST/STM32F1xx/Include/
    Drivers/CMSIS/Include/
    Inc/
)

target_compile_options(ST PRIVATE 
    -Wno-unused-parameter
    -Wno-shadow
)

target_compile_definitions(ST PUBLIC 
    USE_HAL_DRIVER
    STM32F103xB
)

add_executable(${EXE_NAME})

set(ST_CONFIGURATION_SOURCES
    Src/stm32f1xx_hal_msp.c
    Src/system_stm32f1xx.c
    Src/usb_device.c
    Src/usbd_cdc_if.c
    Src/usbd_conf.c
    Src/usbd_desc.c
    Src/syscalls.c
)


target_sources(${EXE_NAME} PRIVATE

    "${ST_CONFIGURATION_SOURCES}"

    startup/startup_stm32f103xb.s

    Src/stm32f1xx_it.c
    Src/main.c

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
)

set_source_files_properties(${ST_CONFIGURATION_SOURCES}
    PROPERTIES COMPILE_OPTIONS -Wno-unused-parameter)

target_compile_options(${EXE_NAME} PRIVATE
    -Wall
    -Wextra
    -Wshadow
)



target_link_options(${EXE_NAME} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld
    -Wl,--gc-sections,--no-warn-rwx-segment
    -specs=nano.specs
    -u _printf_float
)


target_compile_definitions(${EXE_NAME} PUBLIC
    USE_HAL_DRIVER
    STM32F103xB
)

target_link_libraries(${EXE_NAME} PRIVATE
    ST
)

add_custom_target(flash
    openocd -f ${CMAKE_CURRENT_SOURCE_DIR}/${OPENOCD_CFG_NAME} -c "program ${EXE_NAME} reset exit"
    VERBATIM
)

