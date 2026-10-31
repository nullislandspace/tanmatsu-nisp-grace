#!/usr/bin/env bash
# =====================================================================
#  Null Island Space Program  --  host purity, enforced by a grep
# ---------------------------------------------------------------------
#  The orbital and rocket maths under main/sim, main/rocket and main/flight
#  is compiled two ways: into app.so for the badge, and into a plain host
#  binary by tools/nispcheck.c (Part H). That only works if none of it
#  reaches for the engine, the BSP or FreeRTOS, and "we'll be careful" is
#  not a mechanism. So this script is the mechanism: one banned include
#  pattern, three directories, a nonzero exit if a single line matches.
#
#  main/flight is on the list because the flight sim is the same kind of
#  code -- a tick function that takes a state and returns a state. Keeping
#  it pure is what lets Part H case 9 drive it with scripted reentry
#  profiles instead of flying a badge.
# =====================================================================
set -u

BANNED='esp_log\.h|esp_heap_caps\.h|esp_timer\.h|esp_system\.h|esp_err\.h|freertos/|synthengine3d\.h|se_[a-z_]*\.h|pax_gfx\.h|bsp/|graceloader\.h|driver/'
PURE_DIRS=(main/sim main/rocket main/flight)

fail=0
checked=0

for dir in "${PURE_DIRS[@]}"; do
    [ -d "$dir" ] || continue
    while IFS= read -r file; do
        checked=$((checked + 1))
        if grep -nE "^[[:space:]]*#[[:space:]]*include[[:space:]]*[\"<]($BANNED)" "$file" > /dev/null; then
            echo "  IMPURE: $file"
            grep -nE "^[[:space:]]*#[[:space:]]*include[[:space:]]*[\"<]($BANNED)" "$file" | sed 's/^/    /'
            fail=1
        fi
    done < <(find "$dir" \( -name '*.c' -o -name '*.h' \) | sort)
done

if [ $fail -ne 0 ]; then
    echo "hostpurity: FAILED ($checked file(s) checked)"
    exit 1
fi

echo "hostpurity: ok ($checked file(s), no engine or BSP includes)"
