# CLAUDE.md

This file guides Claude Code (claude.ai/code) when working in this repository.

## What this is

APEX flight-computer firmware for an **STM32F411xE** (Cortex-M4F), built from a CubeMX project (`APEX_v1_0.ioc`) with CMake + Ninja and the GNU Arm toolchain. The board carries an IMU (BMI088), a barometer (BMP388), an accelerometer (ADXL375), a magnetometer (LSM303AGR), an IMU (WT901B), GPS, SX127x LoRa/FSK radios, W25Q512 SPI NOR flash, a buzzer and LEDs. USB CDC is the debug/test console.

Source comments are a mix of French and English. Match the language of the file you are editing.

> `.github/copilot-instructions.md` is **out of date**. It describes `Core/Src/drivers`, `Core/Src/utils`, `Core/Src/config` and a `Core_example/` folder, and none of them exist. Trust this file and the code instead.

## Build & flash

```sh
cmake --preset Debug            # configure -> build/Debug (Ninja, cmake/gcc-arm-none-eabi.cmake)
cmake --build --preset Debug    # output: build/Debug/APEX_v1_0.elf (+ .map)
STM32_Programmer_CLI --connect port=swd --download build/Debug/APEX_v1_0.elf -hardRst -rst --start
```

- The `Release` preset is also available. In VS Code, the STM32Cube extension wraps CMake as `cube-cmake`. Use these tasks from `.vscode/tasks.json`: **CMake: clean rebuild**, **CubeProg: Flash project (SWD)** and **Build + Flash**.
- There is no host-side unit-test runner. All tests run **on target**, and their results are printed over USB CDC (see Tests below).

## Selecting what gets built (most important concept)

There is **one** executable, and the application it contains is picked in [cmake/stm32cubemx/CMakeLists.txt](cmake/stm32cubemx/CMakeLists.txt):

- `CMAKE_APEX_PROJECT_PATH` holds a list of `UserProjects/...` folders, all but one commented out. Exactly one must be active. To switch apps, move the comment, then reconfigure.
- The `MX_Application_Src` list in the same file names **every source file explicitly**. There are no globs. Each new `.c` file must be added there, and each new library include dir must be added to `MX_Include_Dirs`. Test sources (`*/test/Src/*.c`) are commented in or out per need. A link error about a missing test symbol usually means its line is commented out.
- The root [CMakeLists.txt](CMakeLists.txt) is template boilerplate. Real configuration lives in the stm32cubemx one.

### UserProject layout

Each project in `UserProjects/{missions,tests/rtos,tests/sequential}/<NAME>/` provides the same 6 files:

| File | Role |
|---|---|
| `Inc/main_config.h` | Compile-time switches: `APEX_CFG_SCHED_SEQ` / `APEX_CFG_SCHED_RTOS` (exactly one), `APEX_CFG_PROFILE_TEST` / `APEX_CFG_PROFILE_MISSION` (exactly one), and `APEX_ENABLE_<DEVICE>` 0/1 per device |
| `Src/main_config.c` | `#error` guards enforcing the mutual exclusions above |
| `Inc/drivers_config.h` / `Src/drivers_config.c` | Global driver instances (`bmi088`, `w25q`, `sx127x_1`, …), their config structs and pin/handle mapping, plus `DRIVERS_CONFIG_init_seq()`. Each section is wrapped in `#if APEX_ENABLE_X`. Keep the header and source sections aligned |
| `Inc/project.h` / `Src/project.c` | Application entry points |

Naming: `SEQ_*` projects use the bare-metal super-loop, and `RTOS_*` projects use FreeRTOS. To make a new project, copy the closest existing one.

## Execution model

[Core/Src/main.c](Core/Src/main.c) initialises the HAL, clocks, all `MX_*` peripherals and USB, then branches on the active project's `main_config.h`:

- **Sequential (`APEX_CFG_SCHED_SEQ`)**: `DRIVERS_CONFIG_init_seq()` → `setup()` once → `loop()` forever in `while(1)`.
- **RTOS (`APEX_CFG_SCHED_RTOS`)**: `osKernelStart()`. `MX_FREERTOS_Init()` in [Core/Src/freertos.c](Core/Src/freertos.c) calls `Init_spi_semaphores()`, and `StartDefaultTask` runs the project's `setup()` in thread context, then exits. **RTOS projects have no `loop()`.** Anything long-lived is a persistent task spawned from `setup()`.

### RTOS rules

- **No heap.** `FreeRTOSConfig.h` forces `configSUPPORT_DYNAMIC_ALLOCATION 0`, and `heap_4.c` is not compiled. All kernel objects must be statically allocated.
- **Tasks go through the static task framework** in [UserLibraries/utils/scheduler/Core/Inc/scheduler.h](UserLibraries/utils/scheduler/Core/Inc/scheduler.h) (read its header doc):
  - `TASK_DECLARE(name, args_t, stack_bytes)` (or `TASK_DECLARE_PERSISTENT`) goes in a driver header and states the contract.
  - `TASK_DEFINE(name) { ... return status; }` goes in the driver source and holds the body.
  - `TASK_POOL(name, n)` / `TASK_POOL_SZ` goes in the **application** (`project.c`) and owns the RAM. Every pool sits in one auditable block.
  - Spawn with `<name>_spawn(&args, &(task_attr_t){ .priority, .ret, .join_bit })`, then call `task_join(...)`. Join bits 1..30 belong to the joiner. Slots are created lazily, then parked and reused. Call these from thread context only, never from an ISR.
- **SPI under RTOS** goes through the DMA + per-bus semaphore wrappers in [Core/Src/spi.c](Core/Src/spi.c) / [Core/Inc/spi.h](Core/Inc/spi.h): `SPI_Begin_DMA_RTOS` (takes the bus and asserts CS) → `SPI_*_DMA_RTOS` → `SPI_End_DMA_RTOS`. Don't call the HAL SPI functions directly from tasks.
- For inter-task data, use `data_topic` (pub/sub ring with per-subscriber cursors and `DT_DATA_LOSS` signalling).

## Code layout

- `Core/`, `Drivers/`, `Middlewares/`, `USB_DEVICE/`, `startup_stm32f411xe.s`, `STM32F411XX_FLASH.ld`: generated by CubeMX. **Only edit inside `/* USER CODE BEGIN */ … /* USER CODE END */` blocks**, because regeneration from the `.ioc` wipes everything else. `Core/{Inc,Src}/Backup/*.bak` are CubeMX backups. Ignore them.
- `UserLibraries/drivers/<chip>/` and `UserLibraries/utils/<lib>/` hold the project-owned code. Each library uses the same layout: `Core/{Inc,Src}` for the library itself and `test/{Inc,Src}` for its on-target test suite.
  - Drivers that have both a bare-metal and an RTOS flavour split them: e.g. `w25q.c` (blocking, sequential) vs `w25q_rtos.c` (DMA + semaphore + `TASK_DECLARE`d wrappers). Test suites follow the same split: `*_seq_test.c` / `*_rtos_test.c`, with shared fixtures in `*_test_common.h`.
- `UserProjects/`: the selectable applications described above.

## Tests

The on-target test harness is in [UserLibraries/utils/test/Core/Inc/test.h](UserLibraries/utils/test/Core/Inc/test.h):

- A suite is a `TEST_case_table_t[]` of `void fn(TEST_case_t *tc)` functions that use `TEST_ASSERT(cond, fmt, ...)`. The macro *returns* on failure, so it can only be used inside test functions.
- A test project's `setup()` does the following: `TEST_configure_cases(table, N, (const bool[]){...})` to enable or disable individual cases → `TEST_perform_cases` → `TEST_wait_host()` (blocks until a terminal opens the CDC port, DTR=1) → `TEST_print_case_result(..., TEST_usb_print, name, desc)`.
- To run a suite, select its `tests/...` project, make sure its test `.c` is uncommented in the CMake source list, then build and flash. Open the USB serial port to see the VT100-formatted report.
- Sequential and RTOS twin suites (e.g. W25Q) share case numbering on purpose. If an RTOS case fails while its sequential twin passes, the fault is in the RTOS layer.

## Conventions

- Feature macros: `APEX_ENABLE_*` and `APEX_CFG_*`, always tested with `#if (X == 1)`.
- Driver status enums (`W25Q_STATE`, `BMI_STATE`, `sx127x_status_t`) are returned through `task_ret_t` (int32) unchanged.
- C11 with GNU extensions. Compiler defines are `USE_HAL_DRIVER` and `STM32F411xE`, plus `DEBUG` in Debug builds.
