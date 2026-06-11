# AART Brake Module — Docker Toolchain (libopencm3)

STM32G051K6U6 firmware built with **libopencm3** — no ST HAL,
no CubeMX, no proprietary dependencies. Everything in the repo.

---

## Why libopencm3?

- Fully open source (LGPL) — the library lives in the repo as a submodule
- No CubeMX, no ST account, no generated code blobs
- Direct register-level control with thin, readable wrappers
- Smaller binaries than HAL
- One important difference on G0: the DMA controller uses a **DMAMUX**
  which must be configured explicitly (see `dma_setup()` in brake_module.c)

---

## Project layout

```
aart-brake-ocm3/
├── Dockerfile              # base → toolchain → cubeprog (optional)
├── docker-compose.yml
├── README.md
├── .dockerignore
├── .gitignore
├── .gitmodules             # libopencm3 as submodule
├── firmware/
│   ├── Core/
│   │   ├── Inc/
│   │   │   └── brake_module.h
│   │   └── Src/
│   │       ├── main.c
│   │       └── brake_module.c
│   └── Makefile
├── scripts/
│   ├── entrypoint.sh
│   ├── build.sh
│   ├── flash.sh
│   └── debug.sh
└── build/                  # created at build time, gitignored
```

---

## Prerequisites

- Docker Desktop for Mac (Apple Silicon)
- Git (for submodule init)
- ST-Link V2/V3 or J-Link probe

---

## First-time setup

### 1. Initialise the libopencm3 submodule

After cloning the repo:

```bash
git submodule update --init --recursive
```

This is only needed on the host if you want to browse the source.
The Docker image clones libopencm3 independently at build time
and pre-builds the G0 library — so the submodule is optional
for Docker users but useful for IDE navigation.

The `.gitmodules` entry looks like this:
```ini
[submodule "libopencm3"]
    path = libopencm3
    url = https://github.com/libopencm3/libopencm3.git
    branch = master
```

### 2. Build the Docker image

```bash
docker build --target toolchain -t aart-brake:toolchain .
```

First build: ~5 minutes (ARM GCC download + libopencm3 compile; OpenOCD from apt is fast).
Subsequent builds: seconds (all layers cached).

---

## Compile the firmware

```bash
# Debug build (default)
docker compose run --rm brake build

# Release build (optimised, no debug symbols)
docker compose run --rm brake build release
```

Or directly without compose:
```bash
docker run --rm \
    -v "$(pwd)/firmware:/workspace/firmware" \
    -v "$(pwd)/build:/workspace/build" \
    aart-brake:toolchain \
    /workspace/scripts/build.sh
```

Output in `./build/`:
| File | Description |
|------|-------------|
| `aart_brake.elf` | Debug ELF with symbols (for GDB) |
| `aart_brake.hex` | Intel HEX (for CubeProgrammer) |
| `aart_brake.bin` | Raw binary |
| `aart_brake.map` | Linker map |

Expected size on STM32G051K6U6 (32K flash, 18K RAM, DAC):
- Text: ~4–6 KB
- BSS: ~100 bytes
- Very comfortable margins.

---

## Flash the firmware

**Recommended on macOS: build in Docker, flash from host.**

Docker Desktop on macOS does not reliably pass USB devices
through to containers. The simplest workflow is:

```bash
# 1. Build inside Docker
docker compose run --rm brake build

# 2. Flash from host using ST-Link tools (install via Homebrew)
brew install stlink
st-flash write build/aart_brake.bin 0x08000000

# OR using openocd on the host
brew install openocd
openocd \
    -f interface/stlink.cfg \
    -f target/stm32g0x.cfg \
    -c "program build/aart_brake.elf verify reset exit"
```

**If you want to flash from inside the container:**
```bash
docker run --rm --privileged \
    -v "$(pwd)/build:/workspace/build" \
    aart-brake:toolchain \
    /workspace/scripts/flash.sh stlink
```
Note: `--privileged` is required for USB access on macOS.

---

## Debug (GDB)

```bash
# Start OpenOCD + GDB inside the container
docker run --rm --privileged -it \
    -v "$(pwd)/build:/workspace/build" \
    -v "$(pwd)/firmware:/workspace/firmware" \
    -p 3333:3333 \
    aart-brake:toolchain \
    /workspace/scripts/debug.sh stlink
```

GDB breaks at `main()` automatically. Useful commands:

```gdb
(gdb) p s_state              # print full BrakeState_t struct
(gdb) p brake_get_mode()     # current mode
(gdb) info locals            # local variables in current frame
(gdb) x/2h &s_adc_buf        # raw ADC DMA buffer (hex halfword)
(gdb) monitor reset halt     # reset MCU and halt
(gdb) continue               # run
(gdb) quit
```

---

## libopencm3 — G0-specific notes

### DMAMUX
The STM32G0 family routes DMA requests through a DMAMUX
peripheral, unlike F0/F1 where the mapping is fixed.
You must configure `DMAMUX1_CxCR` for each DMA channel.

In `brake_module.c`:
```c
// Route ADC1 (request ID 5) to DMA1 Channel 1
dmamux_set_dma_channel_request(DMAMUX1, 0, ADC_DMAMUX_REQ);
```
If you add more DMA channels, set DMAMUX accordingly.

### ADC calibration
On the G0 ADC, `adc_calibrate()` must be called **before**
`adc_power_on()`. libopencm3 handles this correctly if called
in the right order — see `adc_setup()` in brake_module.c.

### Linker script
libopencm3 ships `stm32g051k6.ld` with the correct
flash (32K) and RAM (8K) sizes for this exact part.
No need to write your own.

### ISR names
libopencm3 uses lowercase ISR names with weak defaults:
- `sys_tick_handler` — SysTick (1ms tick)
- `hard_fault_handler` — HardFault (force brake)
- `nmi_handler` — NMI (force brake)
- `dma1_channel1_isr` — DMA complete (ADC results ready)

Override them in your C files — no startup file editing needed.

---

## Optional: STM32CubeProgrammer stage

Download `SetupSTM32CubeProgrammer.linux` from ST (free myST account),
place in `./installers/`, then:

```bash
docker build --target cubeprog -t aart-brake:full .
```

Flash with CubeProgrammer:
```bash
docker run --rm --privileged \
    -v "$(pwd)/build:/workspace/build" \
    aart-brake:full \
    STM32_Programmer_CLI -c port=SWD -w /workspace/build/aart_brake.elf -v -rst
```

---

## Toolchain versions

| Tool | Version |
|------|---------|
| arm-none-eabi-gcc | 13.3.rel1 |
| OpenOCD | 0.12.0 (via apt, Debian Bookworm) |
| libopencm3 | master (pinned at image build) |
| Base OS | Debian Bookworm slim |
