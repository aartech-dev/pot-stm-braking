# AART Slot Car Braking Module

## TODO

    - Verify the design  
    - Get AI to produce KiCad files - again and retrofit the pretty libs etc.

**AART — Adrian & Richard's Technologies**

An [open source](https://creativecommons.org/licenses/by-nc-sa/4.0/) standalone electronic braking controller for slot car racing, supporting brushed and brushless motors. Connects between the hand controller and the track via standard 4mm banana plugs. Compatible with any conventional resistor or transistor hand controller.

> *With thanks to Bob Budge for the original concept.*

---

## What it does

When a driver releases the trigger, a conventional hand controller shorts the track terminals. The motor becomes a generator and this module controls how that energy is absorbed — from a gentle soft brake through to a dead short, with optional positive anti-brake injection and brushless eCom keep-alive.

Three operating modes cover the main competition use cases:

| Mode | Toggle position | Use case |
|------|----------------|----------|
| **A — Positive anti-brake** | A | Brushed motors (high-Kv, Group 12) where hard braking causes deslotting. Pot: CCW = soft brake → centre = dead short → CW = forward voltage injection (0–2V) |
| **B — Reduced braking** | B | Brushed motors needing fine brake adjustment. Full pot range: CCW = ~8Ω → CW = dead short |
| **C — Brushless keep-alive** | C | Brushless motors with eCom. Brake resistance same as mode B, plus a continuously-present low voltage keeps the eCom MCU powered at all times |

**Brake detection** is by voltage sense on the track positive (BLACK) terminal — no mechanical wiper strip contact required. Compatible with any hand controller.

**Brake ramp-in** (modes A and B): configurable exponential onset from 0 to 200ms, giving the "active coast" feel of advanced commercial controllers.

---

## Physical connections

Six 4mm banana sockets — three to the track, three to the hand controller:

| Colour | Signal | Notes |
|--------|--------|-------|
| WHITE | +12–16V always-live rail | From track PSU (40–90A capable). Powers the module. |
| BLACK | Track positive | To car via right-hand braid. Monitored for brake detection. |
| RED | Common negative | Left-hand braid return. |

The module mirrors these three sockets on its output side for the hand controller to plug into.

---

## Repository layout

```
pot-stm-braking/
├── aart-brake-ocm3-fixed/      # Firmware (libopencm3, Docker toolchain)
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── firmware/
│   │   ├── Core/
│   │   │   ├── Inc/brake_module.h   # Constants, pin map, types, API
│   │   │   └── Src/
│   │   │       ├── brake_module.c   # Control logic, ADC/DMA, PWM, flash
│   │   │       └── main.c           # SysTick ISR, fault handlers, main loop
│   │   ├── Makefile
│   │   └── stm32g041j6.ld           # Custom linker script (page 31 reserved)
│   └── scripts/
│       ├── build.sh
│       ├── flash.sh
│       └── debug.sh
├── aart-brake-kicad/               # KiCad schematic and BOM
│   ├── aart_brake.kicad_pro
│   ├── aart_brake.kicad_sch
│   └── aart_brake_bom.csv
├── scripts/                        # Document generator
│   └── gen_doc.js
├── AART_Braking_Module_Design_v8.docx   # Full design document
└── README.md
```

---

## Hardware

**MCU:** STM32G041K6U6 (QFN-32, Cortex-M0+, 32K flash, 8K RAM, 64MHz)

**Key pin assignments:**

| Pin | Signal | Function |
|-----|--------|----------|
| PA0 | WHITE sense | ADC — 16V rail ÷ 18k/4k7 divider → 3.31V max |
| PA1 | BLACK sense | ADC — track voltage ÷ 22k/4k7 divider → 2.82V max. Brake detection. |
| PA2 | Toggle bit 0 | GPIO pull-up — mode select |
| PA3 | Pot wiper | ADC — 10k centre-detent pot |
| PA4 | Toggle bit 1 | GPIO pull-up — mode select |
| PA5 | Capture button | ADC — 10k pull-up + 1k series. ~4095 unpressed, ~372 pressed |
| PA6 | TIM3_CH1 | Brake PWM → GD1 → Q1 (N-ch, **inverted polarity**) |
| PA7 | TIM3_CH2 | Keep-alive PWM → GD2 → Q2 (P-ch, normal polarity) |
| PB6 | Status LED | Blinks 3× on flash save |

**Power components (revised after thermal analysis):**

| Ref | Part | Notes |
|-----|------|-------|
| Q1 | IRLB3034 N-ch D2PAK | Brake FET, 195A/40V, 1.4mΩ Rds(on) |
| Q2 | IRF9Z34N P-ch D2PAK | Keep-alive/anti-brake FET, 18A/55V |
| D1 | **P6KE18CA** (bidirectional) | TVS rail clamp — CA suffix required for both polarities |
| D2/D3 | **SS310** (3A/100V) | FET catch diodes — 100V rating required on 16V rail |
| D4 | **MBRS360 or SS54** | Mode C series protection — most stressed diode, sees repetitive ~46A pulses at 20kHz |
| F1 | 500mA–1A polyfuse | **WHITE rail MCU supply only** — never in the BLACK track path |
| U2 | AMS1117-3.3 SOT-223 | LDO, 12–16V → 3.3V |

> **D4 is essential in mode C.** Without it, Q2's body diode takes the full braking energy pulse every time Q1 fires. See design document section 3.5 for the full thermal analysis.

---

## Firmware

Built with [libopencm3](https://github.com/libopencm3/libopencm3) — no ST HAL, no CubeMX, no proprietary dependencies.

**ADC scan (4 channels, DMA circular):** WHITE rail → BLACK track → pot → capture button

**PWM:** TIM3 at 20kHz. CH1 uses **inverted polarity** (CCR=0 → pin HIGH → Q1 on → dead short). This means the 10kΩ pull-up on GD1's input holds the brake on when the MCU is unpowered — the safe failure mode.

**Brake ramp-in** uses an exponential geometric decay:
```
ccr += (target − ccr) × alpha / 256   (each 1ms tick)
```
`alpha` is derived from the saved ramp time: 0ms = immediate, 200ms = slowest. The shape gives fast initial brake onset (prevents running wide) with a gentler tail (avoids snap-stop).

**Flash storage (page 31, address 0x08007C00):**
```
[63:48] ramp_ms    (0–200ms)
[47:32] ka_ccr     (0–400, keep-alive CCR)
[31:0]  magic      (0xAA270001)
```

---

## Building

### Prerequisites

- Docker Desktop (Apple Silicon Mac — linux/arm64)
- Git

### Build the Docker image

```bash
cd aart-brake-ocm3-fixed
docker build --target toolchain -t aart-brake:toolchain .
```

First build: ~5 minutes (ARM GCC 13.3 download + libopencm3 G0 compile). Subsequent builds are cached and near-instant.

### Compile

```bash
# Debug build
docker compose run --rm brake build

# Release build
docker compose run --rm brake build release
```

Output lands in `./build/` on the host:

| File | Description |
|------|-------------|
| `aart_brake.elf` | Debug ELF with symbols (GDB) |
| `aart_brake.hex` | Intel HEX |
| `aart_brake.bin` | Raw binary |
| `aart_brake.map` | Linker map |

Typical size: ~5–6KB text, well within 32K flash.

### Flash (recommended: build in Docker, flash from host)

Docker Desktop on macOS does not reliably pass USB devices to containers.

```bash
# ST-Link
brew install stlink
st-flash write build/aart_brake.bin 0x08000000

# Or via OpenOCD
brew install openocd
openocd -f interface/stlink.cfg -f target/stm32g0x.cfg \
    -c "program build/aart_brake.elf verify reset exit"
```

### Debug (GDB via OpenOCD)

```bash
docker run --rm --privileged -it \
    -v "$(pwd)/build:/workspace/build" \
    -v "$(pwd)/firmware:/workspace/firmware" \
    -p 3333:3333 \
    aart-brake:toolchain \
    /workspace/scripts/debug.sh stlink
```

GDB connects automatically and breaks at `main()`.

---

## Capture procedure

### Keep-alive voltage (mode C — brushless)

1. Set toggle to **C**
2. Hold capture button — brake is suspended, pot dials Q2 output 0→2V
3. Rotate pot CW slowly until eCom powers up (LED or other indicator)
4. Continue slowly until motor just begins to twitch, then back off slightly
5. Release button — voltage saved to flash, LED blinks 3×
6. Power cycle to verify: eCom should power on immediately

### Brake ramp time (modes A and B)

1. Set toggle to **A** or **B**
2. Hold capture button — pot dials ramp time CCW=0ms → CW=200ms
3. Release button — ramp time saved to flash, LED blinks 3×

Starting points: wing cars 80–150ms, pan cars 0–30ms.

---

## libopencm3 G0-specific notes

The STM32G0 has several differences from F-series that affect libopencm3 usage:

- **DMAMUX** — G0 DMA uses a DMAMUX peripheral. ADC1→DMA1 CH1 requires writing `MMIO32(0x40020800) = 5` (not available via the library on all versions)
- **ADC scan** — `adc_enable_scan_mode()` does not exist on G0; scan is implicit when sequence length > 1
- **ADC symbols** — `ADC_CFGR1_RES_12_BIT` not `ADC_RESOLUTION_12BIT`; `ADC_SMPR_SMPx_039DOT5CYC` not `ADC_SMPR_SMP_39DOT5CYC`
- **Timer reset** — use `rcc_periph_reset_pulse(RST_TIM3)` not `timer_reset()`
- **Linker script** — libopencm3 dropped per-device `.ld` files; `stm32g041j6.ld` is provided in the repo with `MEMORY{}` + `INCLUDE cortex-m-generic.ld`
- **Flash writes** — G0 flash requires 64-bit aligned double-word writes

---

## BSCRA / ISRA legality

Anti-brake and keep-alive voltages are sourced directly from the WHITE (track PSU) rail. No stored energy — no capacitors, batteries, or supercapacitors. This satisfies the direct track power requirement in current BSCRA and ISRA controller regulations. Verify against specific class rules for your event.

---

## Design document

`AART_Braking_Module_Design_v8.docx` covers:

- Physical connections and signal chain
- Three-mode operation with FET truth tables
- Hardware component list with power/thermal analysis
- Pin assignments and voltage divider calculations
- Peripheral configuration (clock, TIM3, ADC, DMA, flash)
- Key design decisions with rationale
- libopencm3 build notes and G0-specific API differences
- Docker toolchain setup
- Pre-commissioning checklist and scope verification sequences
- On-track tuning guidance
- Industry terminology (positive anti-brake vs reduced braking)
- Brake ramp-in theory and setup recommendations
- Power and thermal analysis for all power-carrying components

---

## Toolchain

| Tool | Version |
|------|---------|
| arm-none-eabi-gcc | 13.3.rel1 (ARM official, arm64 native) |
| OpenOCD | 0.12.0 (via Debian Bookworm apt) |
| libopencm3 | master (cloned and built in Docker image) |
| Base OS | Debian Bookworm slim |

---

## Acknowledgements

- Bob Budge — original concept
- Arseny Vakrushev — ESCape32 firmware (related AART work)
- Keith Gibson — brake module contributions
- libopencm3 contributors
