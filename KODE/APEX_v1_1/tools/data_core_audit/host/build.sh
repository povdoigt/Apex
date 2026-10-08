#!/usr/bin/env bash
# Builds and runs the sequential suites (CB, DT, DP) on the host with gcc, to
# iterate fast on pure logic. The TIM5 interrupt cases cannot pass here (no
# interrupt): they report "ISR TIM5 : 0 appels" and are expected to FAIL.
# Usage: build.sh [DT_SRC]   (DT_SRC: alternative data_topic.c, for mutants)
set -euo pipefail
H="$(cd "$(dirname "$0")" && pwd)"
R="$(cd "$H/../../../UserLibraries/utils" && pwd)"
DT_SRC="${1:-$R/data_topic/Core/Src/data_topic.c}"
OUT="${2:-$H/host_tests.exe}"
gcc -std=gnu11 -O1 -g -Wall -Wextra -Wno-unused-parameter -DWITH_DP \
    -I "$H/stubs" -I "$R/circular_buffer/Core/Inc" -I "$R/data_topic/Core/Inc" -I "$R/data_packet/Core/Inc" \
    -I "$R/test/Core/Inc" -I "$R/circular_buffer/test/Inc" -I "$R/data_topic/test/Inc" -I "$R/data_packet/test/Inc" \
    "$R/circular_buffer/Core/Src/circular_buffer.c" "$DT_SRC" \
    "$R/circular_buffer/test/Src/cb_seq_test.c" "$R/data_topic/test/Src/dt_seq_test.c" \
    "$R/data_packet/Core/Src/data_packet.c" "$R/data_packet/test/Src/dp_seq_test.c" \
    "$H/host_main.c" -o "$OUT"
timeout 240 "$OUT" < /dev/null
