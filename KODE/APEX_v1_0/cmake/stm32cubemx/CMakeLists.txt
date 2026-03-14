cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	USE_HAL_DRIVER 
	STM32F411xE
    $<$<CONFIG:Debug>:DEBUG>
)

# Configure project path
set(CMAKE_APEX_PROJECT_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Projects/tests/rtos/RTOS_LED_GPIO
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/Target
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include

    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/ADXL375/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/BMI088/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/BMP388/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/buzzer/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/gps/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/led/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/LSM303AGR/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/sx127x/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/w25q/Core/Inc

    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/ADXL375/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/BMI088/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/BMP388/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/buzzer/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/gps/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/led/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/LSM303AGR/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/sx127x/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/w25q/test/Inc

    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/circular_buffer/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/data_topic/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/flash_stream/Core/Inc
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/iir_filter/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/scheduler/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/telem_format/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/test/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/tools/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/types/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/usb/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/vt100/Core/Inc

    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/circular_buffer/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/data_topic/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/flash_stream/test/Inc
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/iir_filter/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/scheduler/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/telem_format/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/test/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/tools/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/types/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/usb/test/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/vt100/test/Inc

    ${CMAKE_APEX_PROJECT_PATH}/Inc
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/Target/usbd_conf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App/usb_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App/usbd_desc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App/usbd_cdc_if.c

    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/ADXL375/Core/Src/ADXL375.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/BMI088/Core/Src/BMI088.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/BMP388/Core/Src/BMP388.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/buzzer/Core/Src/buzzer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/gps/Core/Src/gps.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/led/Core/Src/led.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/LSM303AGR/Core/Src/LSM303AGR.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/sx127x/Core/Src/sx127x_common.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/sx127x/Core/Src/sx127x_lora.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/sx127x/Core/Src/sx127x_fsk_ook.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/sx127x/Core/Src/sx127x.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/w25q/Core/Src/w25q.c

    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/ADXL375/test/Src/ADXL375_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/BMI088/test/Src/BMI088_seq_test.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/BMP388/test/Src/BMP388_test.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/buzzer/test/Src/buzzer_test.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/gps/test/Src/gps_test.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/led/test/Src/led_test.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/LSM303AGR/test/Src/LSM303AGR_test.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/sx127x/test/Src/sx127x_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/drivers/w25q/test/Src/w25q_seq_test.c

    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/circular_buffer/Core/Src/circular_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/data_topic/Core/Src/data_topic.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/flash_stream/Core/Src/flash_stream.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/iir_filter/Core/Src/iir_filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/scheduler/Core/Src/scheduler.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/telem_format/Core/Src/telem_format.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/test/Core/Src/test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/tools/Core/Src/tools.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/types/Core/Src/types.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/usb/Core/Src/usb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/vt100/Core/Src/vt100.c

    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/circular_buffer/test/Src/cb_seq_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/data_topic/test/Src/dt_seq_test.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/flash_stream/test/Src/flash_stream_test.c
    # # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/iir_filter/test/Src/iir_filter_test.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/scheduler/test/Src/scheduler_test.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/telem_format/test/Src/telem_format_test.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/test/test/Src/test_test.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/tools/test/Src/tools_test.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/types/test/Src/types_test.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/usb/test/Src/usb_test.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../UserLibraries/utils/vt100/test/Src/vt100_test.c

    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/peripherals/adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/peripherals/dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/peripherals/gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/peripherals/i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/peripherals/spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/peripherals/tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/peripherals/usart.c

    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/crc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/freertos.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_hal_timebase_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/syscalls.c

    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32f411xe.s

    ${CMAKE_APEX_PROJECT_PATH}/Src/drivers_config.c
    ${CMAKE_APEX_PROJECT_PATH}/Src/main_config.c
    ${CMAKE_APEX_PROJECT_PATH}/Src/project.c
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/system_stm32f4xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

# Drivers Midllewares


set(USB_Device_Library_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
)
set(FreeRTOS_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS 
    STM32_Drivers
    ${TOOLCHAIN_LINK_LIBRARIES}
    USB_Device_Library
	FreeRTOS
	
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Create USB_Device_Library static library
add_library(USB_Device_Library OBJECT)
target_sources(USB_Device_Library PRIVATE ${USB_Device_Library_Src})
target_link_libraries(USB_Device_Library PUBLIC stm32cubemx)

# Create FreeRTOS static library
add_library(FreeRTOS OBJECT)
target_sources(FreeRTOS PRIVATE ${FreeRTOS_Src})
target_link_libraries(FreeRTOS PUBLIC stm32cubemx)

# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
