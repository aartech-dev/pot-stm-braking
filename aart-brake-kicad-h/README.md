# AART Brake Module — KiCad Schematic

## Files

| File | Description |
|---|---|
| `aart_brake.kicad_pro` | KiCad project file — open this in KiCad |
| `aart_brake.kicad_sch` | Schematic — single sheet, A2 paper |
| `aart_brake_bom.csv` | Bill of materials with footprints and notes |

## Opening

Open `aart_brake.kicad_pro` in **KiCad 7.0 or later**. The schematic
uses only inline symbol definitions (no external library dependencies),
so it should open without any missing symbol warnings.

## Schematic layout

The sheet is arranged left-to-right by signal flow:

```
[Connectors/Inputs] → [MCU U1] → [Gate Drivers GD2/GD1] → [FETs Q2/Q1] → [Track Connectors]
[Power rail J2/F1/D1/U2] runs across the top
[Status LED] sits below the MCU
```

## Key design decisions (see also design document)

**Q2 (brake FET, N-ch low-side):**
TIM3_CH1 uses INVERTED output polarity. CCR=0 → pin HIGH → Q2 fully on → dead short.
The 10kΩ pull-up R2 is on the GD2 *input* (not Q2's gate). When the MCU is unpowered,
R2 holds GD2 input HIGH → Q2 stays on → motor remains shorted = safe default.

**Q1 (keep-alive FET, P-ch high-side):**
IRF9Z34N with gate driven by GD1 (UCC27524). P-channel: gate LOW = FET on.
TIM3_CH2 uses normal polarity: CCR=0 → pin LOW → Q1 off.

**D4 (mode C protection):**
SS34 Schottky in series with Q1 drain output, oriented so current flows
Q1 → D4 → track+. When Q2 hard-brakes (dead short), D4 blocks the reverse
current spike from the track back through Q1's body diode.
D4 is ESSENTIAL for mode C. It can be omitted on mode A/B-only boards.

**Gate driver note:**
The UCC27524 is a dual low-side gate driver. Both outputs (OUTA, OUTB) are
wired in parallel for each FET to double the gate drive current capability.
For Q1 (P-channel), the driver output drives the gate low to turn the FET on.
The gate drive supply for GD2/GD1 is 3.3V (from U2). Verify that 3.3V is
sufficient to fully enhance the chosen FETs (Vgs(th) must be < 3.3V).
Both IRLB3034 and IRF9Z34N are logic-level parts and work correctly at 3.3V.

## Net names

| Net | Description |
|---|---|
| `+12V_RAIL` | Always-live 12.4V from track PSU |
| `+3V3` | 3.3V from LDO U2 |
| `GND` | Common ground |
| `TRACK_POS` | Track positive output (to car) |
| `TRACK_NEG` | Track negative / GND |
| `CTRL_IN` | Hand controller output (0–12.4V) |

## Schematic version

Rev 1.0 — May 2026 — AART (Adrian & Richard's Technologies)
