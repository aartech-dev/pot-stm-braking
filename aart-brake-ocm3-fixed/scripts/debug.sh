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
echo "  AART Brake Module — Debug"
echo "  Probe     : ${PROBE}"
echo "  Image     : ${ELF}"
echo "  GDB server: :3333"
echo "════════════════════════════════════════"

case "${PROBE}" in
    stlink) IFACE="-f interface/stlink.cfg" ;;
    jlink)  IFACE='-f interface/jlink.cfg -c "transport select swd"' ;;
    *)      echo "✗ Unknown probe '${PROBE}'"; exit 1 ;;
esac

# Start OpenOCD in background
eval openocd ${IFACE} \
    -f target/stm32g0x.cfg \
    -c '"init"' \
    -c '"reset halt"' &
OPENOCD_PID=$!

sleep 1

if ! kill -0 "${OPENOCD_PID}" 2>/dev/null; then
    echo "✗ OpenOCD failed — is the probe connected?"
    exit 1
fi

echo "  OpenOCD running (PID ${OPENOCD_PID})"
echo "  Starting GDB..."
echo ""

arm-none-eabi-gdb "${ELF}" \
    -ex "set pagination off" \
    -ex "target extended-remote :3333" \
    -ex "monitor reset halt" \
    -ex "load" \
    -ex "monitor reset halt" \
    -ex "break main" \
    -ex "continue"

kill "${OPENOCD_PID}" 2>/dev/null || true
echo "  Debug session ended."
