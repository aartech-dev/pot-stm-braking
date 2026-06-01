#!/usr/bin/env bash
set -euo pipefail

PROBE="${1:-stlink}"
ELF="${2:-/workspace/build/aart_brake.elf}"

if [ ! -f "${ELF}" ]; then
    echo "✗ No firmware found at ${ELF}"
    echo "  Run: docker compose run --rm brake build"
    exit 1
fi

echo "════════════════════════════════════════"
echo "  AART Brake Module — Flash"
echo "  Probe : ${PROBE}"
echo "  Image : ${ELF}"
echo "════════════════════════════════════════"

case "${PROBE}" in
    stlink)
        openocd \
            -f interface/stlink.cfg \
            -f target/stm32g0x.cfg \
            -c "program ${ELF} verify reset exit"
        ;;
    jlink)
        openocd \
            -f interface/jlink.cfg \
            -c "transport select swd" \
            -f target/stm32g0x.cfg \
            -c "program ${ELF} verify reset exit"
        ;;
    *)
        echo "✗ Unknown probe '${PROBE}' — use stlink or jlink"
        exit 1
        ;;
esac

echo ""
echo "✓ Flash complete. MCU reset and running."
