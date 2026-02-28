# Copilot instructions for APEX_v1_0 (STM32F411 firmware)

## Big picture architecture
- This repository is a **CubeMX-generated STM32 firmware** project built with **CMake + Ninja** (`CMakeLists.txt`, `cmake/stm32cubemx/CMakeLists.txt`).
- Runtime code is organized by domain under `Core/`:
  - `Core/Src/peripherals` + `Core/Inc/peripherals`: MCU peripheral init/wrappers (GPIO, SPI, I2C, USART, TIM, DMA, ADC).
  - `Core/Src/drivers` + `Core/Inc/drivers`: board/sensor/radio drivers (BMI088, BMP388, SX127x, W25Q, etc.).
  - `Core/Src/utils` + `Core/Inc/utils`: shared infra (scheduler/task pool, data topics/pub-sub, circular buffer, USB helpers).
  - `Core/Src/config` + `Core/Inc/config`: compile-time feature/profile switches and global driver instances.
- `Core_example/` is a reference snapshot; treat `Core/` as the active implementation unless explicitly asked otherwise.

## Startup and execution model
- `Core/Src/main.c` currently boots peripherals and runs a **manual while-loop** (BMI088 read + SX127x Tx/Rx + USB CDC logging).
- FreeRTOS scaffolding exists (`Core/Src/freertos.c`, task macros in `utils/scheduler.h`), but scheduler start is commented in `main.c`.
- There are partially wired RTOS task flows in `main.c` and driver files; preserve current active mode unless asked to migrate.

## Configuration conventions
- All feature/profile switches are in `Core/Inc/config/main_config.h`.
- `Core/Src/config/main_config.c` enforces mutually exclusive profile macros at compile time (`#error` guards).
- Global driver objects/config are declared in `Core/Inc/config/drivers_config.h` and instantiated in `Core/Src/config/drivers_config.c`.
- When enabling/disabling a device, keep header macros and `Drivers_Init()` branches aligned.

## Data flow patterns to follow
- Inter-task data uses `data_topic` pub-sub (`Core/Inc/utils/data_topic.h`, `Core/Src/utils/data_topic.c`) with per-subscriber cursors and data-loss signaling (`DT_DATA_LOSS`).
- Short-lived RTOS tasks should use the custom task-pool API in `utils/scheduler.h` (`TASK_POOL_CONFIGURE`, `TASK_POOL_CREATE`, `OS_THREAD_NEW_CSTM`, `osThreadExit_Cstm`) instead of raw dynamic allocation.
- USB CDC transmissions are serialized with a semaphore in `Core/Src/utils/usb.c`; avoid direct concurrent `CDC_Transmit_FS` calls.

## Build, flash, and tooling workflow
- Preferred configure/build uses CMake presets in `CMakePresets.json` (`Debug`/`Release`) and Ninja output in `build/<preset>/`.
- VS Code tasks are the canonical workflow (`.vscode/tasks.json`):
  - `CMake: clean rebuild`
  - `CubeProg: Flash project (SWD)`
  - `Build + Flash` (sequence of rebuild then flash)
- Flashing uses `STM32_Programmer_CLI` with SWD and `${command:cmake.launchTargetPath}` output artifact.
- Toolchain is GCC Arm Embedded via `cmake/gcc-arm-none-eabi.cmake` (Cortex-M4F flags + `STM32F411XX_FLASH.ld`).

## Editing rules for this repo
- Keep CubeMX-generated structure intact (especially peripheral init files and startup/linker assets).
- Prefer adding project logic in `USER CODE` sections or project-owned modules (`Core/Src/drivers`, `Core/Src/utils`, `Core/Src/config`) instead of altering generated boilerplate broadly.
- Mirror existing naming/style (`MX_*_Init`, `*_Init`, uppercase feature macros `APEX_ENABLE_*`).
- `Doc/` contains generated documentation artifacts; do not hand-edit for source changes.