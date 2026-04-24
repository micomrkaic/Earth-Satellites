#!/usr/bin/env bash
# build_wasm.sh — compile earth_orbit.c to WebAssembly for itch.io
#
# Prerequisites:
#   1. Emscripten SDK activated:
#        git clone https://github.com/emscripten-core/emsdk
#        cd emsdk && ./emsdk install 3.1.74 && ./emsdk activate 3.1.74
#        source ./emsdk_env.sh
#        cd ..
#   2. map_data.h generated (run once):
#        python3 gen_map_data.py
#
# Usage:
#   chmod +x build_wasm.sh
#   ./build_wasm.sh          # release build  → web/
#   ./build_wasm.sh --debug  # debug build    → web/

set -euo pipefail

OUT_DIR="web"
SRC="earth_orbit.c"
SHELL="shell.html"

# ── Check prerequisites ─────────────────────────────────────────────────────
if ! command -v emcc &>/dev/null; then
    echo "ERROR: emcc not found. Activate the Emscripten SDK first:"
    echo "  source emsdk/emsdk_env.sh"
    exit 1
fi

if [ ! -f "map_data.h" ]; then
    echo "map_data.h not found — running gen_map_data.py ..."
    python3 gen_map_data.py
fi

mkdir -p "$OUT_DIR"

# ── Compiler flags ──────────────────────────────────────────────────────────
COMMON=(
    -std=c17
    -s USE_SDL=2               # Emscripten's SDL2 port
    -s ALLOW_MEMORY_GROWTH=1   # heap can grow at runtime
    -s INITIAL_MEMORY=67108864 # 64 MB starting heap
    -s ENVIRONMENT=web         # strip Node.js / worker support
    -s SINGLE_FILE=0           # keep .wasm separate (faster loads)
    --shell-file "$SHELL"
    -o "$OUT_DIR/index.html"
)

if [ "${1:-}" = "--debug" ]; then
    echo "Building DEBUG → $OUT_DIR/"
    emcc "$SRC" "${COMMON[@]}" \
        -O0 \
        -g \
        -s ASSERTIONS=2 \
        -s SAFE_HEAP=1 \
        -s STACK_OVERFLOW_CHECK=2
else
    echo "Building RELEASE → $OUT_DIR/"
    emcc "$SRC" "${COMMON[@]}" \
        -O2 \
        -s ASSERTIONS=0
fi

# ── Post-build ──────────────────────────────────────────────────────────────
echo ""
echo "Build complete.  Output files:"
ls -lh "$OUT_DIR"/index.*  "$OUT_DIR"/*.wasm 2>/dev/null || ls -lh "$OUT_DIR"/

echo ""
echo "To test locally (required — browsers block file:// WASM):"
echo "  cd $OUT_DIR && python3 -m http.server 8080"
echo "  open http://localhost:8080/index.html"
echo ""
echo "To package for itch.io:"
echo "  cd $OUT_DIR && zip ../earth_orbit_web.zip index.html index.js *.wasm"
echo "  Upload earth_orbit_web.zip to itch.io"
echo "  Set 'Kind' → HTML, viewport → 1200 × 780"
