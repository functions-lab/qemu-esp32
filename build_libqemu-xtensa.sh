#!/bin/sh
# Builds libqemu-xtensa.so: qemu-system-xtensa's objects linked as a shared
# library, without system/main.c's main().
#
#   ./build_libqemu-xtensa.sh [build-dir]

set -e

src="$(cd "$(dirname "$0")" && pwd)"
build="${1:-$src/build}"
mkdir -p "$build"
cd "$build"

if [ ! -f build.ninja ]; then
  # Every optional feature off, then the ones the ESP32 machines need back on.
  flags="$("$src"/configure --help | perl -ne 'print if s/^  ([a-z][\w-]*) .*/\1/' | tail -n +2 | awk '{print "--disable-"$1}' ORS=' ')"
  # shellcheck disable=SC2086
  "$src"/configure --target-list=xtensa-softmmu --extra-cflags=-fPIC $flags \
    --enable-tcg --enable-system --enable-gcrypt --enable-slirp \
    --enable-debug-info --disable-werror
fi

ninja qemu-system-xtensa

# Relink: capture the executable's link line and turn it into -shared.
rm -f qemu-system-xtensa
ninja -v -d keeprsp qemu-system-xtensa | tail -n 1 > link.cmd
cmd="$(sed -e 's/^\[[0-9]*\/[0-9]*\] //' -e 's/-o qemu-system-xtensa /-shared -o libqemu-xtensa.so /' \
           -e 's/qemu-system-xtensa\.p\/system_main\.c\.o//' link.cmd)"
if [ -f qemu-system-xtensa.rsp ]; then
  sed -i 's/qemu-system-xtensa\.p\/system_main\.c\.o//' qemu-system-xtensa.rsp
fi
eval "$cmd"
ls -l "$build/libqemu-xtensa.so"
