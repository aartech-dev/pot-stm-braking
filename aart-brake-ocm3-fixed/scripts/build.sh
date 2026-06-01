#!/usr/bin/env bash
set -euo pipefail

BUILD_TYPE="${1:-debug}"
JOBS="${NPROC:-$(nproc)}"

echo "════════════════════════════════════════"
echo "  AART Brake Module — Build"
echo "  Type      : ${BUILD_TYPE}"
echo "  Jobs      : ${JOBS}"
echo "  libopencm3: ${LIBOPENCM3_DIR}"
echo "  Toolchain : $(arm-none-eabi-gcc --version | head -1)"
echo "════════════════════════════════════════"

mkdir -p /workspace/build

cd /workspace/firmware

if [ "${BUILD_TYPE}" = "release" ]; then
    make -j"${JOBS}" BUILD_TYPE=release BUILD=/workspace/build
else
    make -j"${JOBS}" BUILD=/workspace/build
fi

echo ""
echo "✓ Build complete — /workspace/build/"
ls -lh /workspace/build/*.elf \
       /workspace/build/*.hex \
       /workspace/build/*.bin 2>/dev/null || true
echo ""
arm-none-eabi-size /workspace/build/aart_brake.elf
