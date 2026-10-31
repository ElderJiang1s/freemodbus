#!/bin/sh
set -eu

test_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
source_dir=${SOURCE_DIR:-"$test_dir/../.."}
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/freemodbus-host.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

# SANITIZERS=none is only for toolchains without sanitizer support.
sanitize_flags=
if [ "${SANITIZERS:-address,undefined}" != none ]; then
    sanitize_flags="-fsanitize=${SANITIZERS:-address,undefined} -fno-sanitize-recover=all"
fi
# Avoid Linux PIE/ASan shadow-address collisions; macOS uses its default linker.
platform_flags=
if [ "$(uname -s)" = Linux ]; then
    platform_flags=-no-pie
fi

# Intentional splitting of compiler flag lists; all paths remain quoted.
# Existing response handlers have unused parameters; keep other warnings fatal.
${CC:-cc} -std=c99 -O1 -g -Wall -Wextra -Werror -Wno-unused-parameter -fno-omit-frame-pointer \
    $sanitize_flags $platform_flags \
    -I"$test_dir/include" -I"$source_dir/port" \
    -I"$source_dir/modbus/include" -I"$source_dir/modbus/rtu" \
    "$test_dir/test_master_rtu.c" \
    "$source_dir/modbus/rtu/mbrtu_m.c" \
    "$source_dir/modbus/rtu/mbcrc.c" \
    "$source_dir/modbus/functions/mbutils.c" \
    "$source_dir/modbus/functions/mbfuncholding_m.c" \
    "$source_dir/modbus/functions/mbfunccoils_m.c" \
    "$source_dir/modbus/functions/mbfuncdisc_m.c" \
    "$source_dir/modbus/functions/mbfuncinput_m.c" \
    -o "$build_dir/test_master_rtu"
"$build_dir/test_master_rtu" "$@"
