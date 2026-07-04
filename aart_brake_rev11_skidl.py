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
  * Multi-pin ICs (U1 STM32G051K6U6, U5 INA210, U6 LM311, U3 CH340N, J_USB) use FUNCTIONAL pin names
    with SEQUENTIAL pad numbers as placeholders. Connectivity (by net) is correct; remap the
    pad numbers to the real footprint pads, or swap in the proper KiCad library symbols.
  * GD1 (anti-brake op-amp) is powered from the WHITE rail (~16V) — a single-supply HV op-amp
    (LM321/LM358-class). It is NOT on +5V. (Confirmed by the SIMetrix sim.)
  * Footprints flagged "CONFIRM" (INA210/LM311 pkg+pinout, STL059 PowerFLAT, diode polarity) need checking.
  * Ideal-diode gate drive (charge pump for Q5/Q6) is an OPEN ITEM; USB diode-OR is standard-topology — verify.
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
# [ideal-diode: TPS1212 retired] CI2t     = P("CI2t", "cap", "Capacitor_SMD:C_0402_1005Metric", [("2","2"), ("1","1")])
# [ideal-diode: TPS1212 retired] CTMR     = P("CTMR", "cap", "Capacitor_SMD:C_0402_1005Metric", [("2","2"), ("1","1")])
# [ideal-diode: TPS1212 retired] C_BST    = P("C_BST", "cap", "Capacitor_SMD:C_0402_1005Metric", [("2","2"), ("1","1")])
C_USB    = P("C_USB", "cap", "Capacitor_SMD:C_0402_1005Metric", [("2","2"), ("1","1")])
C_V3     = P("C_V3", "cap", "Capacitor_SMD:C_0402_1005Metric", [("2","2"), ("1","1")])
# [TM1637 retired] C_v      = P("C_v", "cap", "Capacitor_SMD:C_0402_1005Metric", [("2","2"), ("1","1")])
D1       = P("D1", "diode", "Diode_SMD:D_SMB", [("1","1"), ("2","2")])
D2       = P("D2", "diode", "Diode_SMD:D_SMB", [("2","A"), ("1","K")])
D4       = P("D4", "diode", "Diode_SMD:D_SMB", [("1","K"), ("2","A")])
D5       = P("D5", "LED", "LED_SMD:LED_0805_2012Metric", [("1","K"), ("2","A")])
D_USB    = P("D_USB", "diode", "Diode_SMD:D_SMB", [("1","K"), ("2","A")])
# [TM1637 retired] D_v      = P("D_v", "diode", "Diode_SMD:D_SMB", [("2","A"), ("1","K")])
F1       = P("F1", "PolyFuse_1A", "Resistor_SMD:R_1812_4532Metric", [("1","1"), ("2","2")])
F_USB    = P("F_USB", "PolyFuse_500mA", "Resistor_SMD:R_1812_4532Metric", [("1","1"), ("2","2")])
J1       = P("J1", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("1","1")])
J10      = P("J10", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x4_P2.54mm_Vertical", [("1","1"), ("2","2"), ("3","3"), ("4","4")])
J11      = P("J11", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x3_P2.54mm_Vertical", [("3","3"), ("1","1"), ("2","2")])
J12      = P("J12", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("2","2"), ("1","1")])
J13      = P("J13", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("2","2"), ("1","1")])
J14      = P("J14", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x4_P2.54mm_Vertical", [("1","1"), ("2","2"), ("3","3"), ("4","4")])  # TM1637: 1=CLK 2=DIO 3=GND 4=+3V3
J2       = P("J2", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("1","1")])
J3       = P("J3", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("1","1")])
J4       = P("J4", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("1","1")])
J5       = P("J5", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("1","1")])
J6       = P("J6", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x1_P2.54mm_Vertical", [("1","1")])
J7       = P("J7", "Conn", "Connector_PinHeader_2.54mm:PinHeader_1x3_P2.54mm_Vertical", [("3","3"), ("1","1"), ("2","2")])
J_USB    = P("J_USB", "USB-C", "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12", [("1","GND"), ("2","DP"), ("3","DM"), ("4","VBUS"), ("5","CC1"), ("6","CC2")])
Q1       = P("Q1", "TIP147", "Package_TO_SOT_THT:TO-220-3_Vertical", [("3","E"), ("1","B"), ("2","C")])
Q2       = P("Q2", "TW100N03CC", "Package_SO:Vishay_PowerPAK_SO-8_Single  # TW100N03CC SMD power pkg: CONFIRM", [("3","S"), ("1","G"), ("2","D")])  # low-Rds logic-level (was IRFP250N)
R_SAFE   = P("R_SAFE", "100k", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])  # C3 safe-default: Q2 drain(BLACK_to_track)->gate passive pull-up
D_SAFE   = P("D_SAFE", "MMSZ5242B_12V", "Diode_SMD:D_SOD-123", [("1","K"), ("2","A")])  # C3: Zener gate->source clamp (value: see flag - 8.2V better for logic-level Q2)
Q5       = P("Q5", "STL059N4S8AG", "Package_SO:Vishay_PowerPAK_SO-8_Single  # STL059 PowerFLAT: CONFIRM", [("2","D"), ("3","S"), ("1","G")])  # STL059 PowerFLAT: CONFIRM
Q6       = P("Q6", "STL059N4S8AG", "Package_SO:Vishay_PowerPAK_SO-8_Single  # STL059 PowerFLAT: CONFIRM", [("2","D"), ("3","S"), ("1","G")])  # STL059 PowerFLAT: CONFIRM
# [ideal-diode: TPS1212 retired] Q_GL     = P("Q_GL", "2N2222", "Package_TO_SOT_SMD:SOT-23", [("2","E"), ("2","in"), ("3","out")])
GD3      = P("GD3", "OpAmp", "Package_TO_SOT_SMD:SOT-23-5", [("5","V+"), ("2","V-"), ("4","IN-"), ("3","IN+"), ("1","OUT")])  # anti-brake demand translator (WHITE-ref unity difference); GD1+GD3 = one dual LM7322
R1       = P("R1", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R10      = P("R10", "res", "Resistor_SMD:R_0402_1005Metric", [("2","2"), ("1","1")])
R11      = P("R11", "res", "Resistor_SMD:R_0402_1005Metric", [("2","2"), ("1","1")])
R12      = P("R12", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R2       = P("R2", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R3       = P("R3", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R5       = P("R5", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R8       = P("R8", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R9       = P("R9", "res", "Resistor_SMD:R_0402_1005Metric", [("2","2"), ("1","1")])
# [ideal-diode: TPS1212 retired] RIOC     = P("RIOC", "res", "Resistor_SMD:R_0402_1005Metric", [("2","2"), ("1","1")])
R_B0     = P("R_B0", "res", "Resistor_SMD:R_0402_1005Metric", [("2","2"), ("1","1")])
R_CC1    = P("R_CC1", "res", "Resistor_SMD:R_0402_1005Metric", [("2","2"), ("1","1")])
R_CC2    = P("R_CC2", "res", "Resistor_SMD:R_0402_1005Metric", [("2","2"), ("1","1")])
R_KA1    = P("R_KA1", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R_KA2    = P("R_KA2", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
R_t1     = P("R_t1", "10k", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])  # translator (match 0.1%)
R_t2     = P("R_t2", "10k", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])  # translator (match 0.1%)
R_t3     = P("R_t3", "10k", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])  # translator (match 0.1%)
R_t4     = P("R_t4", "10k", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])  # translator (match 0.1%)
# [TM1637 retired] R_v      = P("R_v", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])
Rsns_ab  = P("Rsns_ab", "shunt", "Resistor_SMD:R_2512_6332Metric", [("1","1"), ("2","2")])
Rsns_sw  = P("Rsns_sw", "1m", "Resistor_SMD:R_2512_6332Metric", [("1","1"), ("2","2")])  # ideal-diode sense shunt (INA210 across); ~1mohm -> +/-0.16A budget
Pot      = P("Pot", "10k_lin", "Potentiometer_THT:Potentiometer_Bourns_3296W_Vertical", [("3","3"), ("2","2"), ("1","1")])
GD1      = P("GD1", "OpAmp", "Package_TO_SOT_SMD:SOT-23-5", [("5","V+"), ("2","V-"), ("4","IN-"), ("3","IN+"), ("1","OUT")])
GD2      = P("GD2", "OpAmp", "Package_TO_SOT_SMD:SOT-23-5", [("5","V+"), ("2","V-"), ("3","IN+"), ("1","OUT"), ("4","IN-")])
U1       = P("U1", "STM32G051K6U6", "Package_DFN_QFN:QFN-32-1EP_5x5mm_P0.5mm_EP3.45x3.45mm", [("1","VDD"), ("2","VDDA"), ("3","VSS"), ("4","VSSA"), ("5","PA0"), ("6","PA1"), ("7","PA3"), ("8","PA4"), ("9","PB3"), ("10","PA5"), ("11","PA9"), ("12","PA10"), ("13","PA13"), ("14","PA14"), ("15","NRST"), ("16","PB8"), ("17","PB0"), ("18","PB1"), ("19","PB4"), ("20","PB6"), ("21","PA6")])
U2       = P("U2", "AMS1117-3.3", "Package_TO_SOT_SMD:SOT-223-3_TabPin2", [("3","IN"), ("2","OUT"), ("1","GND")])
U3       = P("U3", "CH340N", "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm", [("1","VCC"), ("2","GND"), ("3","RXD"), ("4","TXD"), ("5","UD+"), ("6","UD-"), ("7","V3")])
U4       = P("U4", "MCP1700-5002", "Package_TO_SOT_SMD:SOT-23", [("1","IN"), ("3","OUT"), ("2","GND")])
U5       = P("U5", "INA210", "Package_TO_SOT_SMD:SOT-23-6  # SC70/SOT23 INA210: CONFIRM pinout", [("1","IN+"), ("2","IN-"), ("3","GND"), ("4","VS"), ("5","OUT"), ("6","REF")])  # ideal-diode current-sense amp, gain 200
U6       = P("U6", "LM311", "Package_TO_SOT_SMD:SOT-23-5  # CONFIRM", [("1","IN+"), ("2","IN-"), ("3","VCC"), ("4","GND"), ("5","OUT")])  # ideal-diode comparator, 50mV hysteresis
# [ideal-diode] redundant — Rsns_sw is the shunt: RSNS     = P("RSNS", "1m", "Resistor_SMD:R_2512_6332Metric  # CONFIRM power/Kelvin", [("1","1"), ("2","2")])  # reverse-block shunt (INA210 senses across)
R_THR1   = P("R_THR1", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])  # LM311 threshold divider top
R_THR2   = P("R_THR2", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])  # LM311 threshold divider bottom
R_HYS    = P("R_HYS", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])  # LM311 hysteresis (~50mV)
R_CMP    = P("R_CMP", "res", "Resistor_SMD:R_0402_1005Metric", [("1","1"), ("2","2")])  # LM311 open-collector pull-up to +3V3

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
n_SW_FET_IN = Net("SW_FET_IN")  # PWR-BLACK (after sense R): Rsns_sw ~1mohm; INA210 IN+/IN- straddle it (Kelvin) for reverse sense.
n_SW_COMMON_SRC = Net("SW_COMMON_SRC")  # PWR-BLACK (Q5/Q6 common source): back-to-back common source.
n_SW_GATE = Net("SW_GATE")  # ANALOG (common gate): driven by ideal-diode gate driver (charge pump, gated by REV_DETECT) - see GATE DRIVE TODO.
# [ideal-diode: TPS1212 retired] n_U5_BST = Net("U5_BST")  # ANALOG (bootstrap): 100nF typ.
n_REV_DETECT = Net("REV_DETECT")  # DIGITAL: LM311 ideal-diode reverse flag -> Q5/Q6 gate driver + MCU PA6 brake-detect.
n_CSA_OUT = Net("CSA_OUT")  # ANALOG: INA210 output (gain 200) -> LM311 IN+.
n_V_THRESH = Net("V_THRESH")  # ANALOG: LM311 trip threshold (divider + ~50mV hysteresis).
# [ideal-diode: TPS1212 retired] n_U5_INP = Net("U5_INP")  # DIGITAL (on/off command): Normally enabled; glue forces off on reverse. Fast turn-off path.
# [ideal-diode: TPS1212 retired] n_U5_CI2t = Net("U5_CI2t")  # ANALOG: sets OC fault time
# [ideal-diode: TPS1212 retired] n_U5_CTMR = Net("U5_CTMR")  # ANALOG: 
# [ideal-diode: TPS1212 retired] n_U5_IOC = Net("U5_IOC")  # ANALOG: trip ~50-60A
# [ideal-diode: TPS1212 retired] n_PB3_SW_EN = Net("PB3_SW_EN")  # DIGITAL (optional enable): Optional MCU supervisory enable; default enabled. Unused in Rev 11 fw.
n_AB_SENSE = Net("AB_SENSE")  # ANALOG-HV (current sense, ~WHITE): I = (V_WHITE - V_AB_SENSE)/0.1ohm. Common mode ~16V -> GD1 must be a H
n_AB_DEMAND = Net("AB_DEMAND")  # ANALOG-HV (current setpoint = WHITE - 0.30V): GD3 unity difference referenced to WHITE; GD1 regulates shunt to this
n_AB_BASE_DRV = Net("AB_BASE_DRV")  # ANALOG-HV (op-amp -> pass base): GD1 output swings near WHITE; HV op-amp required.
n_AB_INJECT = Net("AB_INJECT")  # PWR (pass collector -> diode): Injection current up to 3A; D4.K -> TRACK_POS.
n_KA_DAC = Net("KA_DAC")  # ANALOG (MCU demand): DAC2 (DAC_OUT2/PA5) DC; R_KA1/R_KA2 divide -> 0-0.30V at KA_DEMAND_LV.
n_KA_DEMAND_LV = Net("KA_DEMAND_LV")  # ANALOG (GND-ref demand 0-0.30V): R_KA1/R_KA2 divider out (DAC2-fed).
n_GD3_INN = Net("GD3_INN")  # ANALOG: GD3 inverting node (R_t1 in, R_t2 fb)
n_GD3_INP = Net("GD3_INP")  # ANALOG: GD3 non-inverting node (~WHITE/2)
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
n_BOOT0 = Net("BOOT0")  # DIGITAL: BOOT0 button node. J12 -> R_B0 (series 10k) -> PA14 (PA14-BOOT0, shares SWCLK; needs nBOOT_SEL=0). No separate BOOT0 ball.
n_TOGGLE_0 = Net("TOGGLE_0")  # DIGITAL: 
n_TOGGLE_1 = Net("TOGGLE_1")  # DIGITAL: 
# [TM1637 retired] VOLTMETER_PWM/VM_OUT/VM_JACK -> PB4 direct CLK, PB8 DIO (no RC/diode)
n_TM1637_CLK = Net("TM1637_CLK")  # DIGITAL: U1 PB4 -> J14.1 (display clock)
n_TM1637_DIO = Net("TM1637_DIO")  # DIGITAL: U1 PB8 -> J14.2 (display data)
# [TM1637 retired] n_VM_OUT = Net("VM_OUT")
# [TM1637 retired] n_VM_JACK = Net("VM_JACK")
n_LED_DRV = Net("LED_DRV")  # DIGITAL: 
n_LED_A = Net("LED_A")  # : D5.K -> GND.
n_USB_VF = Net("USB_VF")  # PWR: F_USB.2->D_USB.A->(K) VWHITE_FUSED
n_POT_TOP = Net("POT_TOP")  # ANALOG: R5.1 on +3V3; pot top = +3V3 via R5
n_P12V_RAIL += J1["1"]
n_P12V_RAIL += J2["1"]
n_P12V_RAIL += D1["1"]
n_P12V_RAIL += F1["1"]
# [ideal-diode: TPS1212 retired] n_P12V_RAIL += U5["VS"]
n_P12V_RAIL += Rsns_ab["1"]
n_P12V_RAIL += R_t3["1"]
n_P12V_RAIL += GD3["V+"]
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
# [ideal-diode: TPS1212 retired] n_GND += U5["GND"]
n_GND += U3["GND"]
n_GND += Q2["S"]
n_GND += D1["2"]
n_GND += D2["A"]
n_GND += R9["2"]
n_GND += R11["2"]
n_GND += Pot["3"]
n_GND += C1["2"]
# [ideal-diode: TPS1212 retired] n_GND += CI2t["2"]
# [ideal-diode: TPS1212 retired] n_GND += CTMR["2"]
# [ideal-diode: TPS1212 retired] n_GND += RIOC["2"]
n_GND += D5["K"]
n_GND += J7["3"]
n_GND += J10["2"]
n_GND += J11["3"]
n_P3V3 += J12["2"]  # BOOT0 button to +3V3 (was GND)
n_GND += J13["2"]
n_GND += J14["3"]  # GND now on J14 pin 3 (TM1637)
n_GND += C2["2"]
n_GND += C3["2"]
n_GND += C4["2"]
n_GND += C4b["2"]
n_GND += C5["2"]
n_GND += C6["2"]
n_GND += C7["2"]
n_GND += C_USB["2"]
n_GND += C_V3["2"]
# [TM1637 retired] n_GND += C_v["2"]
n_GND += R_CC1["2"]
n_GND += R_CC2["2"]
n_GND += R_KA2["2"]
n_GND += R_t4["2"]
n_GND += GD3["V-"]
n_SWDCLK += R_B0["2"]  # series R from BOOT0 button to PA14 (was GND)
# [ideal-diode: TPS1212 retired] n_GND += Q_GL["E"]
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
n_Q2_GATE += R_SAFE["2"]      # C3 back-EMF safe-default pull-up
n_Q2_GATE += D_SAFE["K"]      # C3 gate clamp (cathode at gate)
n_TRACK_POS += R_SAFE["1"]    # R_SAFE top at Q2 drain (BLACK_to_track)
n_GND += D_SAFE["A"]          # Zener anode at source/RED

n_TRACK_POS += J3["1"]
n_TRACK_POS += Q6["D"]
n_TRACK_POS += Q2["D"]
n_TRACK_POS += D2["K"]
n_TRACK_POS += D4["K"]
n_TRACK_POS += R10["1"]

n_CTRL_BLACK_IN += J4["1"]
n_CTRL_BLACK_IN += Rsns_sw["1"]
# [ideal-diode: TPS1212 retired] n_CTRL_BLACK_IN += U5["CS1+"]

n_SW_FET_IN += Rsns_sw["2"]
n_SW_FET_IN += Q5["D"]
# [ideal-diode: TPS1212 retired] n_SW_FET_IN += U5["CS1-"]

n_SW_COMMON_SRC += Q5["S"]
n_SW_COMMON_SRC += Q6["S"]
# [ideal-diode: TPS1212 retired] n_SW_COMMON_SRC += U5["SRC"]
# [ideal-diode: TPS1212 retired] n_SW_COMMON_SRC += C_BST["2"]

n_SW_GATE += Q5["G"]
n_SW_GATE += Q6["G"]
# [ideal-diode: TPS1212 retired] n_SW_GATE += U5["GATE"]

# [ideal-diode: TPS1212 retired] n_U5_BST += U5["BST"]
# [ideal-diode: TPS1212 retired] n_U5_BST += C_BST["1"]

# [ideal-diode: TPS1212 retired] n_U5_IDIR += U5["I_DIR"]
# [ideal-diode: TPS1212 retired] n_U5_IDIR += Q_GL["in"]
# [ideal-diode: TPS1212 retired] n_U5_IDIR += U1["PA6"]    # current-direction brake detect to MCU (CONFIRM I_DIR level <=3.3V; add divider/clamp if higher)

# [ideal-diode: TPS1212 retired] n_U5_INP += U5["INP"]
# [ideal-diode: TPS1212 retired] n_U5_INP += Q_GL["out"]

# [ideal-diode: TPS1212 retired] n_U5_CI2t += U5["CI2t"]
# [ideal-diode: TPS1212 retired] n_U5_CI2t += CI2t["1"]

# [ideal-diode: TPS1212 retired] n_U5_CTMR += U5["CTMR"]
# [ideal-diode: TPS1212 retired] n_U5_CTMR += CTMR["1"]

# [ideal-diode: TPS1212 retired] n_U5_IOC += U5["IOC"]
# [ideal-diode: TPS1212 retired] n_U5_IOC += RIOC["1"]

# [ideal-diode] PB3 now free (was TPS1212 EN): n_PB3_SW_EN += U1["PB3"]
# [ideal-diode: TPS1212 retired] n_PB3_SW_EN += U5["EN"]

n_AB_SENSE += Rsns_ab["2"]
n_AB_SENSE += Q1["E"]
n_AB_SENSE += GD1["IN-"]

n_AB_DEMAND += GD1["IN+"]
n_AB_DEMAND += GD3["OUT"]
n_AB_DEMAND += R_t2["2"]
n_KA_DEMAND_LV += R_KA1["2"]
n_KA_DEMAND_LV += R_KA2["1"]
n_KA_DEMAND_LV += R_t1["1"]

n_AB_BASE_DRV += GD1["OUT"]
n_AB_BASE_DRV += Q1["B"]

n_AB_INJECT += Q1["C"]
n_AB_INJECT += D4["A"]

n_KA_DAC += U1["PA5"]   # PA5 = DAC_OUT2 (analog)
n_KA_DAC += R_KA1["1"]

n_GD3_INN += GD3["IN-"]
n_GD3_INN += R_t1["2"]

n_GD3_INN += R_t2["1"]
n_GD3_INP += GD3["IN+"]
n_GD3_INP += R_t3["2"]
n_GD3_INP += R_t4["1"]

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

# [BOOT0=PA14] n_BOOT0 += U1["BOOT0"]  (BOOT0 reaches PA14 via R_B0; no separate pin)
n_BOOT0 += J12["1"]
n_BOOT0 += R_B0["1"]

n_TOGGLE_0 += U1["PB0"]
n_TOGGLE_0 += J7["1"]

n_TOGGLE_1 += U1["PB1"]
n_TOGGLE_1 += J7["2"]

n_TM1637_CLK += U1["PB4"]
n_TM1637_CLK += J14["1"]

n_TM1637_DIO += U1["PB8"]
n_TM1637_DIO += J14["2"]

n_P3V3 += J14["4"]

n_LED_DRV += U1["PB6"]
n_LED_DRV += R12["1"]

n_LED_A += R12["2"]
n_LED_A += D5["A"]

n_USB_VF += F_USB["2"]
n_USB_VF += D_USB["A"]

n_POT_TOP += R5["2"]
n_POT_TOP += Pot["1"]

# ===== Ideal-diode reverse block (INA210 U5 + LM311 U6) — replaces TPS1212-Q1 =====
# [ideal-diode] redundant — Rsns_sw is the shunt: n_CTRL_BLACK_IN += RSNS["1"]
# [ideal-diode] redundant — Rsns_sw is the shunt: n_SW_FET_IN += RSNS["2"]
n_CTRL_BLACK_IN += U5["IN+"]
n_SW_FET_IN += U5["IN-"]
n_P5V += U5["VS"]
n_GND += U5["GND"]
n_GND += U5["REF"]
n_CSA_OUT += U5["OUT"]
n_CSA_OUT += U6["IN+"]
n_V_THRESH += U6["IN-"]
n_P5V += U6["VCC"]
n_GND += U6["GND"]
n_REV_DETECT += U6["OUT"]
n_P5V += R_THR1["1"]
n_V_THRESH += R_THR1["2"]
n_V_THRESH += R_THR2["1"]
n_GND += R_THR2["2"]
n_REV_DETECT += R_HYS["1"]
n_V_THRESH += R_HYS["2"]
n_REV_DETECT += R_CMP["1"]
n_P3V3 += R_CMP["2"]
n_REV_DETECT += U1["PA6"]   # brake-detect (was TPS1212 I_DIR); LM311 OC pulled to +3V3, CONFIRM polarity
# GATE DRIVE TODO: n_SW_GATE (Q5/Q6 common-source gates) must be lifted above the 0-16V
# BLACK common source. TPS1212 did this with an internal charge pump; the ideal diode needs a
# discrete high-side gate driver / charge pump gated by REV_DETECT. Not yet designed (open item).

# ----------------------------------------------------------------------------- GENERATE
generate_netlist(file_="aart_brake_rev11.net")
print("Wrote aart_brake_rev11.net")
