# tools/: on-target test runner and audit tooling

Scripts used to build, flash and read back any `UserProjects/` application
without a human at the keyboard, plus the host-side and mutation tools used
for the data-core audit.

The folder is named `data_core_audit/` because it was written for the
circular_buffer / data_topic / data_packet audit. The target scripts
(`run_target.sh`, `select_project.sh`, `read_serial.ps1`) are **generic** and
work with any project: they have been used for the BMI088 and W25Q suites and
benches too. Only `run_mutant.sh` and `host/` are specific to the data core.

## Prerequisites

Run everything from **Git Bash** (the Bash tool on this machine). All of the
following must be on `PATH`:

| Tool | Used for |
|---|---|
| `cmake`, `ninja`, `arm-none-eabi-gcc` / `arm-none-eabi-size` | build |
| `STM32_Programmer_CLI` | flash and reset over SWD (ST-LINK) |
| `powershell` | `read_serial.ps1` (opens the USB CDC port) |
| `gcc`, `python` | host runner and host mutation tool only |

You also need the board plugged in through **both** the ST-LINK (SWD) and USB.
The USB CDC console must enumerate as **COM3**. The port is hard-coded in
`read_serial.ps1`, and `run_target.sh` does not override it.

## Layout

| Path | Role |
|---|---|
| `data_core_audit/run_target.sh` | Select → build → flash → capture the USB report of one project |
| `data_core_audit/select_project.sh` | Activate exactly one project in `cmake/stm32cubemx/CMakeLists.txt` |
| `data_core_audit/read_serial.ps1` | Open COM3 with DTR set, read until an end marker or a timeout |
| `data_core_audit/run_mutant.sh` | Same as `run_target.sh`, with the `circular_buffer` critical sections emptied |
| `data_core_audit/host/build.sh` | Build and run the CB / DT / DP sequential suites on the PC with gcc |
| `data_core_audit/host/mutate_registry.py` | Host mutation testing of the `data_topic` subscriber-registry guards |
| `data_core_audit/host/stubs/` | Minimal HAL / CMSIS-RTOS stubs for the host build |
| `data_core_audit/logs/` | **Kept** evidence: dated target reports and host outputs |
| `data_core_audit/tmp/` | Scratch (build, flash and serial logs of the last run). Git-ignored |
| `data_core_audit/PROGRESS.md` | Resume file and journal of the data-core audit (French) |

## Running a project on target

```sh
cd <repo root>
[MARK='<regex>'] bash tools/data_core_audit/run_target.sh <PROJECT_NAME> <timeout_s> <log_name> [--no-build]
```

- `PROJECT_NAME`: folder name under `UserProjects/`, e.g. `RTOS_BMI088_USB` or `SEQ_DT_USB`.
- `log_name`: file stem in `logs/`. Use `YYYY-MM-DD_<PROJECT>[_variant]`, e.g. `2026-10-08_RTOS_BMI088_PERF_debug`.
- `--no-build`: skip selection and build, then flash the existing `build/audit/APEX_v1_0.elf`.

What the script does:

1. **Select.** It calls `select_project.sh`, which comments every `UserProjects/...` line of `CMAKE_APEX_PROJECT_PATH` and uncomments the one named. ⚠️ This **edits `cmake/stm32cubemx/CMakeLists.txt`** and leaves it on the last project run. When you are done, restore the user's selection with `select_project.sh <their project>` and check `git diff cmake/stm32cubemx/CMakeLists.txt`.
2. **Build.** It configures and builds into `build/audit`, which is separate from the user's `build/Debug`. The build is always **Debug**. It prints the ELF size, and warnings filtered to data-core and test files.
3. **Flash.** It flashes over SWD with `STM32_Programmer_CLI ... -hardRst -rst --start`.
4. **Capture.** It starts `read_serial.ps1` (COM3, DTR=1), waits for it to settle, then resets the board over SWD. This way the firmware's single report is sent while the port is already open. `TEST_usb_print` drops output if nobody reads for ~100 ms, so a port opened after the report is lost.
5. **Report.** It writes the report without VT100 codes and `\r` to `logs/<log_name>.txt`, and prints it.
6. **Fallback.** If COM3 is held by another program (a user terminal), it reads the newest `../APEX_v0_2/COM3_*.txt` terminal log written after the flash instead.

### End markers (`MARK`)

The reader stops as soon as the marker regex appears. Pick the marker that matches the project:

| Kind of project | Last line printed | `MARK` |
|---|---|---|
| Test suite (`TEST_print_case_result`) | `23/23 PASS   0 FAIL` | default (no need to set) |
| `RTOS_DT_USB`, `RTOS_DT_STRESS` | `END_OF_REPORT` | default (no need to set) |
| Benchmark (`BENCH_print_csv`: `*_PERF` projects) | `--- CSV END ---` | `MARK='--- CSV END ---'` |
| Anything else | — | a regex on its last line |

With the wrong marker, a bench stops at its first table or runs until the timeout.

### Exit codes

| Code | Meaning |
|---|---|
| `0` | Report captured. **This is not a test verdict**: grep the log for `FAIL`, `ECHEC` or `Erreurs`. |
| `1` | Configure, build or flash error. The tail of `tmp/cmake_config.log`, `tmp/build.log` or `tmp/flash.log` is printed. |
| `2` | Timeout: no marker seen. The partial report is still in the log. |

### Timeouts and long runs

- Suites and benches take well under 2 min, so a timeout of 180–240 s is enough. The tool call itself needs a timeout of about 420 s (build + flash + capture).
- `RTOS_DT_STRESS` runs ×50 suites plus a 10-min endurance, which exceeds the 10-min foreground limit of the Bash tool. Launch it with `run_in_background` and a script timeout of ~1800 s, then read the log when notified.

### Release builds

`run_target.sh` always builds Debug. For a Release run, build by hand into the same directory, then flash with `--no-build`:

```sh
bash tools/data_core_audit/select_project.sh RTOS_BMI088_PERF
cmake -S . -B build/audit -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build/audit
MARK='--- CSV END ---' bash tools/data_core_audit/run_target.sh RTOS_BMI088_PERF 240 <date>_RTOS_BMI088_PERF_release --no-build
```

The next run without `--no-build` reconfigures `build/audit` back to Debug.

## When the report is wrong or never arrives

The firmware may hang or report a failure without a cause, e.g. `init capteur : ECHEC` or a join timeout. In that case, inspect the live core **without resetting it** while it is still in the bad state:

```sh
STM32_Programmer_CLI -c port=swd mode=hotplug -halt -coreReg PC LR -r32 <addr> <n_words> -run
arm-none-eabi-addr2line -f -e build/audit/APEX_v1_0.elf <PC> <LR-1>
```

- `mode=hotplug` attaches without a reset, and `-run` resumes the core afterwards.
- A task that spins forever at normal priority owns the CPU, so a halted PC points straight at the loop. If PC is in the idle task, the blocked task is waiting on a kernel object instead.
- Useful registers: `0xE0001000` (`DWT_CTRL`), `0xE0001004` (`DWT_CYCCNT`), `0xE000EDFC` (`DEMCR`).

Real case (2026-10-08): `BMI088_DelayUs` looped forever. `DWT_CTRL.CYCCNTENA` was still 1 after a reset, but the SWD session had cleared `DEMCR.TRCENA`, so `CYCCNT` was frozen.

Other target pitfalls:
- The report is printed **once**, after `TEST_wait_host()` sees DTR. To get it again, reset the board (re-run with `--no-build`).
- The BMI088 suites expect the board to be **immobile** (|a| ≈ 1 g, low rotation).
- Twin suites (sequential vs RTOS) share case numbers. An RTOS-only failure points at the RTOS layer.

## Mutation testing

- **Target, lock-free mutant**: `run_mutant.sh <PROJECT> <timeout_s> <log>` turns `cb_critical_enter/exit` in `circular_buffer.h` into no-ops for both schedules, runs `run_target.sh`, then restores the header through an `EXIT` trap, even on error. Every concurrency case must **FAIL**. One that passes is too weak to catch a missing critical section. If a run was killed hard, check that `circular_buffer.h` has no `MUTANT` line left (`tmp/circular_buffer.h.orig` holds the original).
- **Host, registry guards**: `python tools/data_core_audit/host/mutate_registry.py` applies each textual mutant of `data_topic.c` into `host/mut/`, rebuilds the host suites against it, and reports killed or surviving mutants. A crash counts as killed. On Windows, `subprocess` must call Git Bash, not the WSL `bash`. The script already does this.

## Host runner

```sh
bash tools/data_core_audit/host/build.sh [alt_data_topic.c] [out.exe]
```

It builds the CB, DT and DP sequential suites with gcc against `host/stubs/` and runs them. Use it for fast iteration on pure logic. The TIM5 interrupt cases print `ISR TIM5 : 0 appels` and **always FAIL** on host. That is expected: only the target proves concurrency.

## Conventions for agents

- Keep evidence: every target or host run that backs a claim goes to `logs/` with a dated name. Do not delete old logs, because they are baselines (e.g. `2026-10-05_stress_full_baseline.txt` for DWT timings).
- For multi-session work, keep a resume file like `PROGRESS.md`: a plan with checkboxes, ticked only with a proof (log, host output, killed mutant), and a dated journal.
- Restore the project selection in `CMakeLists.txt` before handing back, and never touch the user's `build/Debug`.
- When a script misbehaves, read `tmp/` first: `build.log`, `flash.log`, `reset.log` and `serial.out` (`### PORT BUSY`, `### TIMEOUT`).
