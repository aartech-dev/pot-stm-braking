# AART Brake Module — KiCad Schematic (hierarchical, Revision 11)

Firmware Rev 11 design. Open `aart_brake.kicad_pro` in **KiCad 7.0+**.
The schematic is hierarchical: the root sheet (`aart_brake.kicad_sch`)
contains seven sub-sheet symbols. These are annotation-style sheets
(net labels + text callouts; no placed library symbols), so they open
without missing-symbol warnings and document the design for layout.

## Sheets
| # | File | Contents |
|---|---|---|
| root | `aart_brake.kicad_sch` | Top level, component list, design notes |
| 1 | `power_supply.kicad_sch` | F1, D1 TVS, U2 +3V3 LDO, **U4 +5V analog LDO** (TPS1212 VS direct from WHITE, no clamp) |
| 2 | `mcu_inputs.kicad_sch` | U1 STM32G051, **DAC1 PA4 brake, PWM PA5 anti-brake**, pot, toggle PB0/PB1, sense |
| 3 | `debug_headers.kicad_sch` | SWD, UART, BOOT0, NRST |
| 4 | `brake_fet.kicad_sch` | **Q2 IRFP250N** wide-SOA, **GD2 op-amp buffer**, linear (no PWM) |
| 5 | `keepalive_fet.kicad_sch` | **Anti-brake constant-current source: GD1 + Q1 TIP147 + Rsense 0.1ohm, PWM-set 3A** |
| 6 | `track_connectors.kicad_sch` | J1-J6 banana; BLACK split by the reverse switch |
| 7 | `reverse_switch.kicad_sch` | **Q5/Q6 N-ch pair + U5 TPS1212-Q1 (reverse block + I2t OCP + 5us SCP)** |

## Latest design changes (Revision 11)
- **Reverse-block driver is now the TI TPS1212-Q1** (VS from WHITE), replacing the LTC7001 +
  INA241 + comparator + DZ1/RZ1 clamp. It keeps the Q5/Q6 pair but adds I2t overcurrent, fast
  5us short-circuit protection, bidirectional IMON, and FLT diagnostics, with an RSNS sense
  resistor. Reverse response is ~6-10us (slower than a 0.75us ideal-diode controller) - accepted
  here in exchange for the integrated protection. TI part vs the ADI LTC7001.
- **Anti-brake is now a constant-current source** (3A), not a voltage follower. GD1 op-amp +
  Q1 TIP147 PNP Darlington + 0.1ohm sense; demand is a 50kHz PWM (level-shift + RC filter).
  Bounds the current at stall (a near-short on the start line) and matches real controllers.
  Anti-brake net renamed DAC2_Q1_KA -> KA_PWM_DEMAND; DAC_CH2 freed, PA5 = timer PWM.

## What changed across earlier schematic revisions (now unified as Revision 11)
This schematic was two design generations behind. Updated to match the
current design document (v29) and firmware Rev 11:

- **DAC, not PWM.** PA4/PA5 dual DAC drive op-amp buffers in linear mode.
  TIM3 PWM gate drive retired. (Voltmeter still uses PB4 PWM.)
- **Op-amp buffers, not UCC27524** for both gate channels.
- **Anti-brake = op-amp/Darlington voltage follower** (Q1 TIP122). DAC2 sets
  the injection voltage directly: 0V = off, ~2V = max. Polarity inverted vs
  the retired P-channel floating-supply scheme (DAC_KA_OFF is now 0).
  Retired: IRF9Z34N, ZD1/R_Z1/C_Z1, LM321 floating supply.
- **Brake FET IRLB3034 → IRFP250N** (wide-SOA, TO-247, heatsink). Both Q1 and
  Q2 run linear at 10-15W and **require heatsinks**.
- **Reverse-block pass element (new sheet 7):** two N-channel (Q5/Q6,
  common-source) + LTC7001 charge-pump driver **powered from WHITE, not the
  anode**, with INA241 lossless current sense + comparator on INP. Replaces the
  retired LTC4412 ideal-diode pass gate. ~27-47mV insertion drop at 40A.
- **Toggle PB0/PB1**, capture button removed, **PB3 reserved** (optional
  reverse-switch enable; unused in Rev 11).
- **+5V analog rail (U4)** added for the followers, CSA, and comparator.

## Net names
| Net | Description |
|---|---|
| `+12V_RAIL` | WHITE, always-live 12-16V from track PSU |
| `+3V3` | MCU rail (U2) |
| `+5V` | Analog rail (U4): followers, INA241, comparator |
| `GND` | Common ground (RED rail / 0V) |
| `CTRL_BLACK_IN` | Controller BLACK in (J4) → reverse switch input |
| `TRACK_POS` | Track BLACK (J3); switch output; motor +; injection node |
| `DAC1_Q2_BRAKE` / `DAC2_Q1_KA` | DAC outputs to the two stages |
| `PB3_SW_EN` | Optional reverse-switch supervisory enable |

## Note on the .kicad_pro
Child sheets are referenced by the sheet symbols in the root `.kicad_sch`;
KiCad assigns page numbers/UUIDs on first open. The hierarchy will populate
automatically. These sheets are documentation-grade (labels + callouts);
place real symbols/footprints in KiCad for board layout.

Revision 11 — 19 June 2026 — AART (Adrian & Richard's Technologies)
