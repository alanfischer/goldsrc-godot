#!/usr/bin/env bash
set -euo pipefail
# Build the goldsrc GDExtension. Cross-compiles via Zig; lipos a universal on macOS.
#
# Usage:
#   ./build.sh                       # default platforms (macos linux windows android)
#   ./build.sh macos                 # only the listed platform(s)
#   ./build.sh linux windows
#
# Outputs → addons/goldsrc/bin/lib<name>.<platform>.template_<cfg>.<arch>.<ext>
# Kept intentionally parallel to hop-godot/build.sh — edit both together.

# ── Per-extension config (the only real difference between the two scripts) ───
ADDON="goldsrc"          # addons/<ADDON>/bin
LIB="libgoldsrc"         # output library basename
DEFAULT_TARGETS=(macos linux windows android)

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CMAKE_DIR="$SCRIPT_DIR/cmake"
BIN_DIR="$SCRIPT_DIR/addons/$ADDON/bin"

die() { echo "ERROR: $*" >&2; exit 1; }
command -v cmake &>/dev/null || die "'cmake' not found — install it first"
command -v zig   &>/dev/null || die "'zig' not found — install with: brew install zig"

# Configure + build one variant, reusing the build dir so CMake can do a fast
# incremental rebuild. Only wipes when a previous run used a different generator
# (e.g. the old Ninja-based build) that CMake refuses to reconfigure over. Uses
# the default (Make) generator on purpose: Ninja + the Zig Windows toolchain
# leaks -femit-implib into compile steps and fails.
#   build <build-subdir> <toolchain-file|""> <Debug|Release> [extra -D cmake args...]
build() {
    local dir="$1" toolchain="$2" type="$3"; shift 3
    local gdt=template_release; [ "$type" = Debug ] && gdt=template_debug
    local bdir="$SCRIPT_DIR/$dir"
    echo ""; echo "--- $dir ($type) ---"
    if [ -f "$bdir/CMakeCache.txt" ] && ! grep -q 'CMAKE_GENERATOR:INTERNAL=Unix Makefiles' "$bdir/CMakeCache.txt"; then
        rm -rf "$bdir"   # stale non-Make generator — CMake can't reconfigure over it
    fi
    local args=(-S "$SCRIPT_DIR" -B "$bdir" -DCMAKE_BUILD_TYPE="$type" -DGODOTCPP_TARGET="$gdt" "$@")
    [ -n "$toolchain" ] && args+=(-DCMAKE_TOOLCHAIN_FILE="$CMAKE_DIR/$toolchain")
    cmake "${args[@]}"
    cmake --build "$bdir" --parallel
}

# Fuse the arm64 + x86_64 slices of one config into a universal dylib.
universal() {
    local type="$1"
    command -v lipo &>/dev/null || { echo "    (lipo unavailable — skipping universal)"; return; }
    lipo -create "$BIN_DIR/$LIB.macos.$type.arm64.dylib" "$BIN_DIR/$LIB.macos.$type.x86_64.dylib" \
        -output "$BIN_DIR/$LIB.macos.$type.universal.dylib"
    echo "    wrote $LIB.macos.$type.universal.dylib"
}

build_macos() {
    build build-macos-arm64-debug    toolchain-macos-arm64.cmake   Debug    # editor / dev runs
    build build-macos-arm64-release  toolchain-macos-arm64.cmake   Release
    build build-macos-x86_64-debug   toolchain-macos-x86_64.cmake  Debug
    build build-macos-x86_64-release toolchain-macos-x86_64.cmake  Release
    universal template_debug
    universal template_release
}

build_linux()   { build build-linux-x86_64-release toolchain-linux-x86_64.cmake Release; }
# GODOTCPP_USE_STATIC_CPP=OFF: godot-cpp's Windows static block adds a bare -static
# that conflicts with -shared under Zig ("-femit-implib allowed only when building a
# DLL"). Zig statically links its own runtime, so the DLL stays self-contained anyway.
build_windows() { build build-windows-x86_64-release toolchain-windows-x86_64.cmake Release -DGODOTCPP_USE_STATIC_CPP=OFF; }

# Android arm64 (Meta Quest 3) — needs the NDK. Auto-detect under ANDROID_HOME.
build_android() {
    if [ -z "${ANDROID_NDK:-}" ] && [ -n "${ANDROID_HOME:-}" ]; then
        ANDROID_NDK=$(ls -d "$ANDROID_HOME/ndk/"* 2>/dev/null | sort -V | tail -1)
    fi
    if [ -z "${ANDROID_NDK:-}" ] || [ ! -d "${ANDROID_NDK:-}" ]; then
        echo ""; echo "--- android (SKIPPED — set ANDROID_NDK to build for Quest) ---"
        return
    fi
    export ANDROID_NDK   # visible to compiler-detection sub-invocations
    build build-android-arm64-debug   toolchain-android-arm64.cmake Debug
    build build-android-arm64-release toolchain-android-arm64.cmake Release
}

targets=("$@"); [ $# -eq 0 ] && targets=("${DEFAULT_TARGETS[@]}")
for t in "${targets[@]}"; do
    case "$t" in
        macos|linux|windows|android) "build_$t" ;;
        *) die "unknown target '$t' (macos|linux|windows|android)" ;;
    esac
done

echo ""; echo "==> Done. Binaries in: $BIN_DIR"
ls -lh "$BIN_DIR"
