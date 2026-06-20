#!/usr/bin/env python3
"""
AART Slot-Car Braking Module — Revision 11
SKiDL netlist generator.  Reconstructed from the design doc + BOM + the SIMetrix
anti-brake sim (AntibrakeConstCurrent.wxsch).

USAGE:
    pip install skidl
    python aart_brake_rev11_skidl.py
  -> writes aart_brake_rev11.net  (KiCad netlist; import in Pcbnew: File > Import Netlist)

IMPORTANT NOTES / CONFIRM BEFORE LAYOUT:
  * Parts are created self-contained (no KiCad symbol-lib lookup), so this runs anywhere.
  * 2- and 3-pin parts (R,C,D,LED,Q,LDO,op-amp,fuse) use real pad numbers and are layout-ready.
  * Multi-pin ICs (U1 STM32G051K6U6, U5 TPS1212-Q1, U3 CH340N, J_USB) use FUNCTIONAL pin names
    with SEQUENTIAL pad numbers as placeholders. Connectivity (by net) is correct; remap the
    pad numbers to the real footprint pads, or swap in the proper KiCad library symbols.
  * GD1 (anti-brake op-amp) is powered from the WHITE rail (~16V) — a single-supply HV op-amp
    (LM321/LM358-class). It is NOT on +5V. (Confirmed by the SIMetrix sim.)
  * Footprints flagged "CONFIRM" (TPS1212 VQFN-23, STL059 PowerFLAT, diode polarity) need checking.
  * Glue (Q_GL: U5 I_DIR -> INP) and the USB diode-OR are at standard-topology level — verify.
  * Any KICAD_SYMBOL_DIR / fp-lib-table warnings are harmless (we use self-contained parts).
"""
from skidl import Part, Pin, Net, generate_netlist, SKIDL

def P(ref, value, footprint, pins):
    """Create a self-contained part: pins = list of (padnum, pinname)."""
    prt = Part(tool=SKIDL, name=value, tag=ref, ref_prefix=ref.rstrip("0123456789_") or "U",
               footprint=footprint.split("  #")[0].strip(),
               pins=[Pin(num=str(n), name=str(nm)) for n, nm in pins])
    prt.ref = ref
    prt.value = value
    return prt

# ----------------------------------------------------------------------------- PARTS
C1       = P("C1", "cap", "Capacitor_SMD:C_0402_1005Metric", [("1","1"), ("2","2")])
C2       = P("C2", "cap", "Capacitor_SMD:C_0402_1005Metric", [("1","1"), ("2","2")])
C3       = P("C3", "cap", "Capacitor_SMD:C_0402_1005Metric", [("1","1"), ("2","2")])
C4       = P("C4", "cap", "Capacitor_SMD:C_0402_1005Metric", [("1","1"), ("2","2")])
C4b      = P("C4b", "cap", "Capacitor_SMD:C_0402_1005Metric", [("1","1"), ("2","2")])
C5       = P("C5", "cap", "Capacitor_SMD:C_0402_1005Metric", [("1","1"), ("2","2")])
C6       = P("C6", "cap", "Capacitor_SMD:C_0402_1005Metric", [("1","1"), ("2","2")])
C7       = P("C7", "cap", "Capacitor_SMD:C_0402_1005Metric", [("1","1"), ("2","2")])
CI2t     = P("CI2t", "cap", "Capacitor_SMD:C_0402_1005Metric", [("2","2"), ("1","1")])
CTMR     = P("CTMR", "cap", "Capacitor_SMD:C_0402_1005Metric", [("2","2"), ("1","1")])
C_BST    = P("C_BST", "cap", "Capacitor_SMD:C_0402_1005Metric", [("2","2"), ("1","1")])
C_KA     = P("C_KA", "470n", "Capacitor_SMD:C_0402_1005Metric", [("1","1"), ("2","2")])
C_USB    = P("C_USB", "cap", "Capacitor_SMD:C_0402_1005Metric", [("2","2"), ("1","1")])
C_V3     = P("C_V3", "cap", "Capacitor_SMD:C_0402_1005Metric", [("2","2"), ("1","1")])
C_v      = P("C_v", "cap", "Capacitor_SMD:C_0402_1005Metric", [("2","2"), ("1","1")])
D1       = P("D1", "diode", "Diode_SMD:D_SMB", [("1","1"), ("2","2")])
D2       = P("D2", "diode", "Diode_SMD:D_SMB", [("2","A"), ("1","K")])
D4       = P("D4", "diode", "Diode_SMD:D_SMB", [("1","K"), ("2","A")])
D5       = P("D5", "LED", "LED_SMD:LED_0805_2012Metric", [("1","K"), ("2","A")])
D_USB    = P("D_USB", "diode", "Diode_SMD:D_SMB", [("1","K"), ("2","A")])
D_v      = P("D_v", "diode", "Diode_SMD:D_SMB", [("2","A"), ("1","K")])
F1       = P("F1", "PolyFuse_1A", "Resistor_SMD:R_1812_4532Metric", [("1","1"), ("2","2")])
F_USB    = P("F_USB", "PolyFuse_500mA", "Resistor_SMD:R_1812_4532Metric", [("1","1"), ("2","2")])
J1       = P("J1", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("1","1")])
J10      = P("J10", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x4_P2.54mm_Vertical", [("1","1"), ("2","2"), ("3","3"), ("4","4")])
J11      = P("J11", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x3_P2.54mm_Vertical", [("3","3"), ("1","1"), ("2","2")])
J12      = P("J12", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("2","2"), ("1","1")])
J13      = P("J13", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("2","2"), ("1","1")])
J14      = P("J14", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x3_P2.54mm_Vertical", [("2","2"), ("1","1")])
J2       = P("J2", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("1","1")])
J3       = P("J3", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("1","1")])
J4       = P("J4", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("1","1")])
J5       = P("J5", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("1","1")])
J6       = P("J6", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("1","1")])
J7       = P("J7", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x3_P2.54mm_Vertical", [("3","3"), ("1","1"), ("2","2")])
J_USB    = P("J_USB", "USB-C", "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12", [("1","GND"), ("2","DP"), ("3","DM"), ("4","VBUS"), ("5","CC1"), ("6","CC2")])
Q1       = P("Q1", "TIP147", "Package_TO_SOT_THT:TO-220-3_Vertical", [("3","E"), ("1","B"), ("2","C")])
Q2       = P("Q2", "IRFP250N", "Package_TO_SOT_THT:TO-247-3_Vertical", [("3","S"), ("1","G"), ("2","D")])
Q5       = P("Q5", "STL059N4S8AG", "Package_SO:Vishay_PowerPAK_SO-8_Single  # STL059 PowerFLAT: CONFIRM", [("2","D"), ("3","S"), ("1","G")])  # STL059 PowerFLAT: CONFIRM
Q6       = P("Q6", "STL059N4S8AG", "Package_SO:Vishay_PowerPAK_SO-8_Single  # STL059 PowerFLAT: CONFIRM", [("2","D"), ("3","S"), ("1","G")])  # STL059 PowerFLAT: CONFIRM
Q_GL     = P("Q_GL", "2N2222", "Package_TO_SOT_SMD:SOT-23", [("2","E"), ("2","in"), ("3","out")])
Q_LS     = P("Q_LS", "2N2222", "Package_TO_SOT_SMD:SOT-23", [("3","C"), ("1","B"), ("2","E")])
R1       = P("R1", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R10      = P("R10", "res", "Resistor_SMD:R_0402_1005Metric", [("2","2"), ("1","1")])
R11      = P("R11", "res", "Resistor_SMD:R_0402_1005Metric", [("2","2"), ("1","1")])
R12      = P("R12", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R2       = P("R2", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R3       = P("R3", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R5       = P("R5", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R8       = P("R8", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R9       = P("R9", "res", "Resistor_SMD:R_0402_1005Metric", [("2","2"), ("1","1")])
RIOC     = P("RIOC", "res", "Resistor_SMD:R_0402_1005Metric", [("2","2"), ("1","1")])
R_B0     = P("R_B0", "res", "Resistor_SMD:R_0402_1005Metric", [("2","2"), ("1","1")])
R_CC1    = P("R_CC1", "res", "Resistor_SMD:R_0402_1005Metric", [("2","2"), ("1","1")])
R_CC2    = P("R_CC2", "res", "Resistor_SMD:R_0402_1005Metric", [("2","2"), ("1","1")])
R_KA1    = P("R_KA1", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R_KAbase = P("R_KAbase", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R_KAe    = P("R_KAe", "res", "Resistor_SMD:R_0402_1005Metric", [("2","2"), ("1","1")])
R_v      = P("R_v", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
Rsns_ab  = P("Rsns_ab", "shunt", "Resistor_SMD:R_2512_6332Metric", [("1","1"), ("2","2")])
Rsns_sw  = P("Rsns_sw", "shunt", "Resistor_SMD:R_2512_6332Metric", [("1","1"), ("2","2")])
Pot      = P("Pot", "10k_lin", "Potentiometer_THT:Potentiometer_Bourns_3296W_Vertical", [("3","3"), ("2","2"), ("1","1")])
GD1      = P("GD1", "OpAmp", "Package_TO_SOT_SMD:SOT-23-5", [("5","V+"), ("2","V-"), ("4","IN-"), ("3","IN+"), ("1","OUT")])
GD2      = P("GD2", "OpAmp", "Package_TO_SOT_SMD:SOT-23-5", [("5","V+"), ("2","V-"), ("3","IN+"), ("1","OUT"), ("4","IN-")])
U1       = P("U1", "STM32G051K6U6", "Package_DFN_QFN:QFN-32-1EP_5x5mm_P0.5mm_EP3.45x3.45mm", [("1","VDD"), ("2","VDDA"), ("3","VSS"), ("4","VSSA"), ("5","PA0"), ("6","PA1"), ("7","PA3"), ("8","PA4"), ("9","PB3"), ("10","PA5"), ("11","PA9"), ("12","PA10"), ("13","PA13"), ("14","PA14"), ("15","NRST"), ("16","BOOT0"), ("17","PB0"), ("18","PB1"), ("19","PB4"), ("20","PB6")])
U2       = P("U2", "AMS1117-3.3", "Package_TO_SOT_SMD:SOT-223-3_TabPin2", [("3","IN"), ("2","OUT"), ("1","GND")])
U3       = P("U3", "CH340N", "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm", [("1","VCC"), ("2","GND"), ("3","RXD"), ("4","TXD"), ("5","UD+"), ("6","UD-"), ("7","V3")])
U4       = P("U4", "MCP1700-5002", "Package_TO_SOT_SMD:SOT-23", [("1","IN"), ("3","OUT"), ("2","GND")])
U5       = P("U5", "TPS1212-Q1", "Package_DFN_QFN:QFN-24-1EP_4x4mm_P0.5mm_EP2.6x2.6mm  # 23-pin: CONFIRM", [("1","VS"), ("2","GND"), ("3","CS1+"), ("4","CS1-"), ("5","SRC"), ("6","GATE"), ("7","BST"), ("8","I_DIR"), ("9","INP"), ("10","CI2t"), ("11","CTMR"), ("12","IOC"), ("13","EN")])  # 23-pin: CONFIRM

# ----------------------------------------------------------------------------- NETS
n_P12V_RAIL = Net("+12V_RAIL")  # PWR (WHITE, raw/unfused, high-current): WHITE rail. Anti-brake current source draws up to 3A here; must NOT go
n_VWHITE_FUSED = Net("VWHITE_FUSED")  # PWR (WHITE after F1 -> LDOs): Low-current LDO supply. USB VBUS OR's in here via D_USB for reflash-on
n_P3V3 = Net("+3V3")  # PWR (MCU rail): 3.3V logic rail.
n_P5V = Net("+5V")  # PWR (brake-buffer rail): 5V analog rail. NOTE: only GD2 (brake buffer) lives here now; GD1 move
n_GND = Net("GND")  # PWR (common 0V / RED return): Star/plane ground. Keep the 40A BLACK/brake return (RED) copper separa
n_RAIL_SENSE = Net("RAIL_SENSE")  # ANALOG (WHITE divider -> ADC): R8=15k(top to +12V_RAIL), R9=4k7(bottom to GND). 16V -> <3.3V.
n_BLACK_SENSE = Net("BLACK_SENSE")  # ANALOG (BLACK divider -> ADC): R10=22k(top to TRACK_POS), R11=4k7(bottom to GND). Tap on TRACK side o
n_POT_WIPER = Net("POT_WIPER")  # ANALOG (pot -> ADC): Single control pot. CCW=min, CW=max.
n_BRAKE_DAC = Net("BRAKE_DAC")  # ANALOG (DAC1 -> brake buffer): On MCU reset DAC is Hi-Z; R2 pulls GD2 input to +5V -> Q2 defaults to 
n_BRAKE_GATE_DRV = Net("BRAKE_GATE_DRV")  # ANALOG (buffer out): GD2 = unity buffer; may need gain >1 to reach ~5V gate from 3.3V DAC f
n_Q2_GATE = Net("Q2_GATE")  # ANALOG: 
n_TRACK_POS = Net("TRACK_POS")  # PWR-BLACK (track + / motor + / injection): The shared BLACK-to-track node: switch output, brake high side, anti-b
n_CTRL_BLACK_IN = Net("CTRL_BLACK_IN")  # PWR-BLACK (controller in): Controller BLACK in. Separate net from TRACK_POS (switch is between th
n_SW_FET_IN = Net("SW_FET_IN")  # PWR-BLACK (after sense R): Rsns_sw ~0.5-0.75mohm. CS1+/CS1- straddle it (Kelvin).
n_SW_COMMON_SRC = Net("SW_COMMON_SRC")  # PWR-BLACK (Q5/Q6 common source): Common source = TPS1212 SRC. Bootstrap cap C_BST sits BST->SRC.
n_SW_GATE = Net("SW_GATE")  # ANALOG (common gate): TPS1212 GATE drives both gates (2A sink for fast turn-off).
n_U5_BST = Net("U5_BST")  # ANALOG (bootstrap): 100nF typ.
n_U5_IDIR = Net("U5_IDIR")  # DIGITAL (reverse flag): I_DIR -> glue -> INP. CONFIRM polarity & glue topology vs TPS1212 refe
n_U5_INP = Net("U5_INP")  # DIGITAL (on/off command): Normally enabled; glue forces off on reverse. Fast turn-off path.
n_U5_CI2t = Net("U5_CI2t")  # ANALOG: sets OC fault time
n_U5_CTMR = Net("U5_CTMR")  # ANALOG: 
n_U5_IOC = Net("U5_IOC")  # ANALOG: trip ~50-60A
n_PB3_SW_EN = Net("PB3_SW_EN")  # DIGITAL (optional enable): Optional MCU supervisory enable; default enabled. Unused in Rev 11 fw.
n_AB_SENSE = Net("AB_SENSE")  # ANALOG-HV (current sense, ~WHITE): I = (V_WHITE - V_AB_SENSE)/0.1ohm. Common mode ~16V -> GD1 must be a H
n_AB_DEMAND = Net("AB_DEMAND")  # ANALOG-HV (current setpoint, ref to WHITE): Per sim: demand node pulled to WHITE by R_KA1/C_KA, pulled down by Q_L
n_AB_BASE_DRV = Net("AB_BASE_DRV")  # ANALOG-HV (op-amp -> pass base): GD1 output swings near WHITE; HV op-amp required.
n_AB_INJECT = Net("AB_INJECT")  # PWR (pass collector -> diode): Injection current up to 3A; D4.K -> TRACK_POS.
n_KA_PWM = Net("KA_PWM")  # DIGITAL (MCU demand): Filtered to DC by the level-shift+RC. R_KAbase.2 -> Q_LS.B.
n_KA_LS_BASE = Net("KA_LS_BASE")  # ANALOG: 
n_KA_LS_EMIT = Net("KA_LS_EMIT")  # ANALOG: Emitter degeneration (R1 in sim).
n_UART_TX = Net("UART_TX")  # DIGITAL: 
n_UART_RX = Net("UART_RX")  # DIGITAL: 
n_USB_DP = Net("USB_DP")  # USB: 
n_USB_DM = Net("USB_DM")  # USB: 
n_VBUS = Net("VBUS")  # PWR (USB 5V): F_USB.2 -> D_USB.A -> VWHITE_FUSED.
n_USB_CC1 = Net("USB_CC1")  # USB: 
n_USB_CC2 = Net("USB_CC2")  # USB: 
n_U3_V3 = Net("U3_V3")  # ANALOG: CH340N internal reg decoupling.
n_SWDIO = Net("SWDIO")  # DIGITAL: 
n_SWDCLK = Net("SWDCLK")  # DIGITAL: 
n_NRST = Net("NRST")  # DIGITAL: 
n_BOOT0 = Net("BOOT0")  # DIGITAL: 
n_TOGGLE_0 = Net("TOGGLE_0")  # DIGITAL: 
n_TOGGLE_1 = Net("TOGGLE_1")  # DIGITAL: 
n_VOLTMETER_PWM = Net("VOLTMETER_PWM")  # ANALOG (PWM->DC meter): R_v+C_v -> D_v -> J14.1.
n_VM_OUT = Net("VM_OUT")  # ANALOG: 
n_VM_JACK = Net("VM_JACK")  # ANALOG: 
n_LED_DRV = Net("LED_DRV")  # DIGITAL: 
n_LED_A = Net("LED_A")  # : D5.K -> GND.
n_USB_VF = Net("USB_VF")  # PWR: F_USB.2->D_USB.A->(K) VWHITE_FUSED
n_POT_TOP = Net("POT_TOP")  # ANALOG: R5.1 on +3V3; pot top = +3V3 via R5
n_P12V_RAIL += J1["1"]
n_P12V_RAIL += J2["1"]
n_P12V_RAIL += D1["1"]
n_P12V_RAIL += F1["1"]
n_P12V_RAIL += U5["VS"]
n_P12V_RAIL += Rsns_ab["1"]
n_P12V_RAIL += R_KA1["1"]
n_P12V_RAIL += C_KA["1"]
n_P12V_RAIL += GD1["V+"]
n_P12V_RAIL += R8["1"]

n_VWHITE_FUSED += F1["2"]
n_VWHITE_FUSED += U2["IN"]
n_VWHITE_FUSED += U4["IN"]
n_VWHITE_FUSED += C1["1"]
n_VWHITE_FUSED += C2["1"]
n_VWHITE_FUSED += D_USB["K"]

n_P3V3 += U2["OUT"]
n_P3V3 += C3["1"]
n_P3V3 += C4["1"]
n_P3V3 += U1["VDD"]
n_P3V3 += U1["VDDA"]
n_P3V3 += C5["1"]
n_P3V3 += C6["1"]
n_P3V3 += C7["1"]
n_P3V3 += R1["1"]
n_P3V3 += R5["1"]
n_P3V3 += U3["VCC"]
n_P3V3 += J10["1"]

n_P5V += U4["OUT"]
n_P5V += C4b["1"]
n_P5V += GD2["V+"]
n_P5V += R2["1"]

n_GND += J5["1"]
n_GND += J6["1"]
n_GND += U1["VSS"]
n_GND += U1["VSSA"]
n_GND += U2["GND"]
n_GND += U4["GND"]
n_GND += GD1["V-"]
n_GND += GD2["V-"]
n_GND += U5["GND"]
n_GND += U3["GND"]
n_GND += Q2["S"]
n_GND += D1["2"]
n_GND += D2["A"]
n_GND += R9["2"]
n_GND += R11["2"]
n_GND += Pot["3"]
n_GND += C1["2"]
n_GND += CI2t["2"]
n_GND += CTMR["2"]
n_GND += RIOC["2"]
n_GND += D5["K"]
n_GND += J7["3"]
n_GND += J10["2"]
n_GND += J11["3"]
n_GND += J12["2"]
n_GND += J13["2"]
n_GND += J14["2"]
n_GND += C2["2"]
n_GND += C3["2"]
n_GND += C4["2"]
n_GND += C4b["2"]
n_GND += C5["2"]
n_GND += C6["2"]
n_GND += C7["2"]
n_GND += C_USB["2"]
n_GND += C_V3["2"]
n_GND += C_v["2"]
n_GND += R_CC1["2"]
n_GND += R_CC2["2"]
n_GND += R_KAe["2"]
n_GND += R_B0["2"]
n_GND += Q_GL["E"]
n_GND += J_USB["GND"]

n_RAIL_SENSE += U1["PA0"]
n_RAIL_SENSE += R8["2"]
n_RAIL_SENSE += R9["1"]

n_BLACK_SENSE += U1["PA1"]
n_BLACK_SENSE += R10["2"]
n_BLACK_SENSE += R11["1"]

n_POT_WIPER += U1["PA3"]
n_POT_WIPER += Pot["2"]

n_BRAKE_DAC += U1["PA4"]
n_BRAKE_DAC += GD2["IN+"]
n_BRAKE_DAC += R2["2"]

n_BRAKE_GATE_DRV += GD2["OUT"]
n_BRAKE_GATE_DRV += GD2["IN-"]
n_BRAKE_GATE_DRV += R3["1"]

n_Q2_GATE += R3["2"]
n_Q2_GATE += Q2["G"]

n_TRACK_POS += J3["1"]
n_TRACK_POS += Q6["D"]
n_TRACK_POS += Q2["D"]
n_TRACK_POS += D2["K"]
n_TRACK_POS += D4["K"]
n_TRACK_POS += R10["1"]

n_CTRL_BLACK_IN += J4["1"]
n_CTRL_BLACK_IN += Rsns_sw["1"]
n_CTRL_BLACK_IN += U5["CS1+"]

n_SW_FET_IN += Rsns_sw["2"]
n_SW_FET_IN += Q5["D"]
n_SW_FET_IN += U5["CS1-"]

n_SW_COMMON_SRC += Q5["S"]
n_SW_COMMON_SRC += Q6["S"]
n_SW_COMMON_SRC += U5["SRC"]
n_SW_COMMON_SRC += C_BST["2"]

n_SW_GATE += Q5["G"]
n_SW_GATE += Q6["G"]
n_SW_GATE += U5["GATE"]

n_U5_BST += U5["BST"]
n_U5_BST += C_BST["1"]

n_U5_IDIR += U5["I_DIR"]
n_U5_IDIR += Q_GL["in"]

n_U5_INP += U5["INP"]
n_U5_INP += Q_GL["out"]

n_U5_CI2t += U5["CI2t"]
n_U5_CI2t += CI2t["1"]

n_U5_CTMR += U5["CTMR"]
n_U5_CTMR += CTMR["1"]

n_U5_IOC += U5["IOC"]
n_U5_IOC += RIOC["1"]

n_PB3_SW_EN += U1["PB3"]
n_PB3_SW_EN += U5["EN"]

n_AB_SENSE += Rsns_ab["2"]
n_AB_SENSE += Q1["E"]
n_AB_SENSE += GD1["IN-"]

n_AB_DEMAND += GD1["IN+"]
n_AB_DEMAND += R_KA1["2"]
n_AB_DEMAND += C_KA["2"]
n_AB_DEMAND += Q_LS["C"]

n_AB_BASE_DRV += GD1["OUT"]
n_AB_BASE_DRV += Q1["B"]

n_AB_INJECT += Q1["C"]
n_AB_INJECT += D4["A"]

n_KA_PWM += U1["PA5"]
n_KA_PWM += R_KAbase["1"]

n_KA_LS_BASE += R_KAbase["2"]
n_KA_LS_BASE += Q_LS["B"]

n_KA_LS_EMIT += Q_LS["E"]
n_KA_LS_EMIT += R_KAe["1"]

n_UART_TX += U1["PA9"]
n_UART_TX += U3["RXD"]
n_UART_TX += J11["1"]

n_UART_RX += U1["PA10"]
n_UART_RX += U3["TXD"]
n_UART_RX += J11["2"]

n_USB_DP += U3["UD+"]
n_USB_DP += J_USB["DP"]

n_USB_DM += U3["UD-"]
n_USB_DM += J_USB["DM"]

n_VBUS += J_USB["VBUS"]
n_VBUS += F_USB["1"]
n_VBUS += C_USB["1"]

n_USB_CC1 += J_USB["CC1"]
n_USB_CC1 += R_CC1["1"]

n_USB_CC2 += J_USB["CC2"]
n_USB_CC2 += R_CC2["1"]

n_U3_V3 += U3["V3"]
n_U3_V3 += C_V3["1"]

n_SWDIO += U1["PA13"]
n_SWDIO += J10["3"]

n_SWDCLK += U1["PA14"]
n_SWDCLK += J10["4"]

n_NRST += U1["NRST"]
n_NRST += R1["2"]
n_NRST += J13["1"]

n_BOOT0 += U1["BOOT0"]
n_BOOT0 += J12["1"]
n_BOOT0 += R_B0["1"]

n_TOGGLE_0 += U1["PB0"]
n_TOGGLE_0 += J7["1"]

n_TOGGLE_1 += U1["PB1"]
n_TOGGLE_1 += J7["2"]

n_VOLTMETER_PWM += U1["PB4"]
n_VOLTMETER_PWM += R_v["1"]

n_VM_OUT += R_v["2"]
n_VM_OUT += C_v["1"]
n_VM_OUT += D_v["A"]

n_VM_JACK += D_v["K"]
n_VM_JACK += J14["1"]

n_LED_DRV += U1["PB6"]
n_LED_DRV += R12["1"]

n_LED_A += R12["2"]
n_LED_A += D5["A"]

n_USB_VF += F_USB["2"]
n_USB_VF += D_USB["A"]

n_POT_TOP += R5["2"]
n_POT_TOP += Pot["1"]

# ----------------------------------------------------------------------------- GENERATE
generate_netlist(file_="aart_brake_rev11.net")
print("Wrote aart_brake_rev11.net")
