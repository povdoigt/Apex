#!/usr/bin/env bash
# Usage: run_mutant.sh <PROJECT_NAME> <TIMEOUT_S> <LOG_NAME>
# Temporarily turns cb_critical_enter/exit into no-ops (both schedules),
# runs the project on target, then restores circular_buffer.h whatever happens.
# Every concurrency case must FAIL on this mutant; one that passes is too weak.
set -uo pipefail
T="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$T/../.." && pwd)"
H="$ROOT/UserLibraries/utils/circular_buffer/Core/Inc/circular_buffer.h"
mkdir -p "$T/tmp"
cp "$H" "$T/tmp/circular_buffer.h.orig"
restore() { cp "$T/tmp/circular_buffer.h.orig" "$H"; echo "[mutant] circular_buffer.h restored"; }
trap restore EXIT

awk '
  /^static inline cb_critical_t cb_critical_enter\(void\) \{/ { print; print "    return 0u; /* MUTANT: no masking */"; skip = 1; next }
  /^static inline void cb_critical_exit\(cb_critical_t saved\) \{/ { print; print "    (void)saved; /* MUTANT: no masking */"; skip = 1; next }
  skip && /^\}/ { skip = 0; print; next }
  skip { next }
  { print }
' "$T/tmp/circular_buffer.h.orig" > "$H"
grep -n "MUTANT" "$H"
bash "$T/run_target.sh" "$1" "$2" "$3"
