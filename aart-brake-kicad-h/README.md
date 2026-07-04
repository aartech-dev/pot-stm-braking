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
| 1 | `power_supply.kicad_sch` | F1, D1 TVS, U2 +3V3 LDO, **U4 +5V analog LDO** (powers INA210/LM311 ideal diode; gate-drive charge pump from WHITE) |
| 2 | `mcu_inputs.kicad_sch` | U1 STM32G051, **DAC1 PA4 brake, DAC2 PA5 anti-brake**, pot, toggle PB0/PB1, sense |
| 3 | `debug_headers.kicad_sch` | SWD, UART, BOOT0, NRST |
| 4 | `brake_fet.kicad_sch` | **Q2 TW100N03CC** low-Rds logic-level, **GD2 op-amp buffer**, linear (no PWM) |
| 5 | `keepalive_fet.kicad_sch` | **Anti-brake constant-current source: GD1 + Q1 TIP147 + Rsense 0.1ohm, DAC2-set 3A** |
| 6 | `track_connectors.kicad_sch` | J1-J6 banana; BLACK split by the reverse switch |
| 7 | `reverse_switch.kicad_sch` | **Q5/Q6 N-ch pair + U5 INA210 + U6 LM311 ideal diode (~2us, ~0.16A trip)** |

## Latest design changes (Revision 11)
- **Reverse-block driver is now an INA210 + LM311 ideal diode**, replacing the TPS1212-Q1.
  An INA210 current-sense amp (gain 200, +5V) across Rsns_sw (~1mohm) feeds an LM311 comparator
  (~50mV hysteresis, ~0.16A trip) that turns the Q5/Q6 pair off on reverse current in ~2us (sim) -
  ~13x lower glitch energy than the TPS1212, so a smaller eCom brownout when the brake contact closes.
  This drops the TPS1212 I2t/SCP/IMON automotive protection (open item: the brake event is brief and
  the rest of the design is unprotected, but the pass FETs lose their own fault protection). OPEN ITEMS:
  a high-side gate driver / charge pump (from WHITE) must lift the gates above the 0-16V BLACK common
  source; the ~2us speed and the 26V INA210 CM ceiling need bench validation.
- **Anti-brake is now a constant-current source** (3A), not a voltage follower. GD1 op-amp +
  Q1 TIP147 PNP Darlington + 0.1ohm sense; demand is DC from DAC2 (R_KA1/R_KA2 divider + GD3 LM7321 translator).
  Bounds the current at stall (a near-short on the start line) and matches real controllers.
  Anti-brake net is KA_DAC_DEMAND; PA5 = DAC2 (DAC_CH2) direct DC demand.

## What changed across earlier schematic revisions (now unified as Revision 11)
This schematic was two design generations behind. Updated to match the
current design document (v29) and firmware Rev 11:

- **DAC, not PWM.** PA4/PA5 dual DAC drive op-amp buffers in linear mode.
  TIM3 PWM gate drive retired. (PB4 now drives the TM1637 digital display; the PB4 voltmeter PWM is retired.)
- **Op-amp buffers, not UCC27524** for both gate channels.
- **Anti-brake = constant-current source** (Q1 TIP147 + GD1 HV op-amp + Rsense
  0.1ohm), superseding the earlier TIP122 voltage follower. DAC2 sets the current
  demand via R_KA1/R_KA2 + GD3 LM7321: code 0 = off, full = 3A (DAC_KA_OFF is now 0).
  Retired: IRF9Z34N, ZD1/R_Z1/C_Z1, LM321 floating supply; TIP122 follower.
- **Brake FET → Q2 TW100N03CC** (low-Rds logic-level, full-on for brushless;
  IRFP250N = brushed-only alt). Q1 (TIP147) runs linear ~10-15W and **requires a heatsink**.
- **Reverse-block pass element (sheet 7):** two N-channel (Q5/Q6, common-source)
  + U5 INA210 + U6 LM311 ideal-diode controller (ICs on +5V), with Rsns_sw (~1mohm)
  sense, ~0.16A reverse trip, ~50mV hysteresis, ~2us. No I2t/SCP/IMON; gate-drive charge
  pump from WHITE (open item). Supersedes the TPS1212-Q1. ~87mV at 40A.
- **Toggle PB0/PB1**, capture button removed, **PB3 free** (the ideal diode
  has no enable input; the TPS1212 EN is gone).
- **+5V analog rail (U4)** for the GD2 brake gate buffer (GD1 runs on WHITE).

## Net names
| Net | Description |
|---|---|
| `+12V_RAIL` | WHITE, always-live 12-16V from track PSU |
| `+3V3` | MCU rail (U2) |
| `+5V` | Analog rail (U4): GD2 brake gate buffer |
| `GND` | Common ground (RED rail / 0V) |
| `CTRL_BLACK_IN` | Controller BLACK in (J4) → reverse switch input |
| `TRACK_POS` | Track BLACK (J3); switch output; motor +; injection node |
| `DAC1_Q2_BRAKE` / `KA_DAC_DEMAND` | DAC1 brake; DAC2 anti-brake demand |
| `PB3` | Free GPIO (was reverse-switch enable; ideal diode has no enable) |

## Note on the .kicad_pro
Child sheets are referenced by the sheet symbols in the root `.kicad_sch`;
KiCad assigns page numbers/UUIDs on first open. The hierarchy will populate
automatically. These sheets are documentation-grade (labels + callouts);
place real symbols/footprints in KiCad for board layout.

Revision 11 — 19 June 2026 — AART (Adrian & Richard's Technologies)
