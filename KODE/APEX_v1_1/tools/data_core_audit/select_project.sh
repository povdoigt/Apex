#!/usr/bin/env bash
# Usage: select_project.sh <PROJECT_NAME>
# Comments every UserProjects line of the CMAKE_APEX_PROJECT_PATH block, then
# uncomments the one ending with /<PROJECT_NAME>. Fails if it is not found.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
F="$ROOT/cmake/stm32cubemx/CMakeLists.txt"
NAME="$1"
awk -v name="$NAME" '
  BEGIN { inblk = 0; found = 0 }
  /^set\(CMAKE_APEX_PROJECT_PATH/ { inblk = 1; print; next }
  inblk && /^\)/ { inblk = 0; print; next }
  inblk && /UserProjects\// {
    line = $0
    sub(/^[ \t]*#[ \t]*/, "", line); sub(/^[ \t]*/, "", line)
    if (line ~ ("/" name "$")) { print "    " line; found++ }
    else                       { print "    # " line }
    next
  }
  { print }
  END { if (found != 1) { print "select_project: " found " match(es) for " name > "/dev/stderr"; exit 1 } }
' "$F" > "$F.tmp"
mv "$F.tmp" "$F"
grep -n "UserProjects/" "$F" | grep -v "#" | head -3
