#!/usr/bin/env bash
# Usage: [MARK=regex] run_target.sh <PROJECT_NAME> <TIMEOUT_S> <LOG_NAME> [--no-build]
# Selects the project, builds it in build/audit (separate from the user's
# build/Debug), flashes it over SWD and captures the USB CDC report into
# logs/<LOG_NAME>.txt.
# The report is read on COM3 directly (read_serial.ps1, DTR set). If COM3 is
# held by a terminal, the report is taken from the newest terminal log in
# ../APEX_v0_2/COM3_<date>.txt instead.
set -euo pipefail
T="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$T/../.." && pwd)"
TERM_LOGS="$(cd "$ROOT/.." && pwd)/APEX_v0_2"
NAME="$1"; TMO="$2"; LOG="$3"; NOBUILD="${4:-}"
BUILD="$ROOT/build/audit"
MARK="${MARK:-[0-9]+/[0-9]+ PASS +[0-9]+ FAIL|END_OF_REPORT}"
mkdir -p "$T/logs" "$T/tmp"

if [ "$NOBUILD" != "--no-build" ]; then
  bash "$T/select_project.sh" "$NAME"
  cd "$ROOT"
  cmake -S . -B "$BUILD" -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug \
        > "$T/tmp/cmake_config.log" 2>&1 || { tail -30 "$T/tmp/cmake_config.log"; exit 1; }
  cmake --build "$BUILD" > "$T/tmp/build.log" 2>&1 || { grep -E "error|Error" "$T/tmp/build.log" | head -40; exit 1; }
  grep -E "warning" "$T/tmp/build.log" | grep -E "circular_buffer|data_topic|data_packet|_test\.c|test_irq|project\.c|freertos\.c|_stress" | head -20 || true
  arm-none-eabi-size "$BUILD/APEX_v1_0.elf" | tail -1
fi

touch "$T/tmp/flash_marker"
sleep 1
STM32_Programmer_CLI --connect port=swd --download "$BUILD/APEX_v1_0.elf" -hardRst -rst --start > "$T/tmp/flash.log" 2>&1 \
      || { tail -20 "$T/tmp/flash.log"; exit 1; }
grep -E "File download complete|Error" "$T/tmp/flash.log" | head -3

# 1) COM3 directly. The firmware prints its report once, when it sees DTR,
# and drops it if nobody reads within ~100 ms (TEST_usb_print). A handle
# opened during the re-enumeration that follows the flash can lose it: the
# reader is opened first, left to settle, then the board is reset over SWD
# so that the tests start with the port already open.
OUT_WIN="$(cygpath -w "$T/logs/$LOG.txt" 2>/dev/null || echo "$T/logs/$LOG.txt")"
PS1_WIN="$(cygpath -w "$T/read_serial.ps1" 2>/dev/null || echo "$T/read_serial.ps1")"
sleep 3
powershell -NoProfile -ExecutionPolicy Bypass -File "$PS1_WIN" -Out "$OUT_WIN" -TimeoutSec "$TMO" -Until "$MARK" \
     > "$T/tmp/serial.out" 2>&1 &
READER=$!
sleep 3
STM32_Programmer_CLI --connect port=swd -hardRst > "$T/tmp/reset.log" 2>&1 || tail -5 "$T/tmp/reset.log"
if wait "$READER"; then
  if ! grep -q "### PORT BUSY" "$T/tmp/serial.out"; then
    tr -d '\r' < "$T/logs/$LOG.txt" | sed 's/^\xEF\xBB\xBF//' > "$T/tmp/log.clean" && mv "$T/tmp/log.clean" "$T/logs/$LOG.txt"
    cat "$T/logs/$LOG.txt"
    grep -q "### TIMEOUT" "$T/tmp/serial.out" && exit 2
    exit 0
  fi
fi

# 2) Fallback: the user's terminal holds COM3 and logs every session.
echo "### COM3 busy: reading the terminal logs in $TERM_LOGS ###"
deadline=$(( $(date +%s) + TMO ))
while [ "$(date +%s)" -lt "$deadline" ]; do
  for f in $(find "$TERM_LOGS" -maxdepth 1 -name 'COM3_*.txt' -newer "$T/tmp/flash_marker" 2>/dev/null | sort); do
    if sed 's/\x1b\[[0-9;]*[A-Za-z]//g' "$f" | grep -Eq "$MARK"; then
      sleep 1
      sed 's/\x1b\[[0-9;]*[A-Za-z]//g' "$f" | tr -d '\r' > "$T/logs/$LOG.txt"
      cat "$T/logs/$LOG.txt"
      exit 0
    fi
  done
  sleep 2
done
echo "### TIMEOUT after $TMO s: no report ###"
exit 2
