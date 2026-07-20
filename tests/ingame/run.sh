#!/usr/bin/env bash
set -uo pipefail
# Run goldsrc's in-engine suites: the Godot-facing node classes (GoldSrcBSP, GoldSrcSPR,
# GoldSrcMDL, GoldSrcWAD, VisibilityManager), which the standalone C++ tests in tests/
# cannot reach because they need a running engine.
#
# Usage:
#   ./run.sh                # all suites
#   ./run.sh bsp_query      # just suites/suite_bsp_query.gd
#
# Override the engine with GODOT=/path/to/Godot. Requires ../../build.sh to have been run
# — without the built extension the runner stops at "GoldSrcBSP missing".
#
# Deliberately parallel to goldnet-godot/tests/gdscript/run.sh; edit both together.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# The suites run against the repo root, not against tests/ingame, so that res://addons and
# res://maps resolve the same way they do in the real project.
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# A wedged run must not hang CI. macOS has no coreutils `timeout`, so this is rolled by
# hand. It is load-bearing rather than defensive: a GDScript parse error leaves headless
# Godot running with no main loop and no output, forever.
TIMEOUT_S="${TIMEOUT_S:-120}"

GODOT="${GODOT:-}"
if [ -z "$GODOT" ] || [ ! -x "$GODOT" ]; then
    for candidate in \
        "/Applications/Godot.app/Contents/MacOS/Godot" \
        "$HOME/Applications/Godot.app/Contents/MacOS/Godot" \
        "$(command -v godot 2>/dev/null || true)"; do
        if [ -n "$candidate" ] && [ -x "$candidate" ]; then GODOT="$candidate"; break; fi
    done
fi
if [ ! -x "${GODOT:-}" ]; then
    echo "Godot binary not found. Set GODOT=/path/to/Godot" >&2
    exit 127
fi

ARGS=(--headless --path "$PROJECT_DIR" res://tests/ingame/main.tscn)
if [ $# -gt 0 ]; then
    ARGS+=(-- --suite "$1")
fi

log="$(mktemp)"
trap 'rm -f "$log"' EXIT

"$GODOT" "${ARGS[@]}" > "$log" 2>&1 &
pid=$!

waited=0
while kill -0 "$pid" 2>/dev/null; do
    if [ "$waited" -ge "$TIMEOUT_S" ]; then
        echo "  (watchdog: killing the run after ${TIMEOUT_S}s)"
        kill -9 "$pid" 2>/dev/null
        break
    fi
    sleep 1
    waited=$((waited + 1))
done
wait "$pid" 2>/dev/null
rc=$?

# Godot writes its banner, the extension's own load chatter and any engine errors here
# too; show only the suite report on success, and everything on failure.
if [ "$rc" -eq 0 ]; then
    grep -E "^(PASS|ingame:)" "$log"
else
    sed 's/^/  | /' "$log"
    echo ""
    echo "ingame suites FAILED (exit $rc)"
fi
exit "$rc"
