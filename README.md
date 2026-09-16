# EMARI — Embedded Control System for Infrared-Assisted Metabolite Extraction

**Bare-metal C/C++ firmware and custom PCB design for a closed-loop laboratory instrument.**

An ESP32-S3 controller running a non-blocking state machine that manages PID thermal regulation, dual closed-loop motor control, phase-angle AC power modulation, and a real-time HMI — concurrently, without an RTOS.

`ESP32-S3` · `Bare-metal C/C++` · `PID Control` · `SPI / I²C` · `KiCad` · `PT100 / MAX31865`

---

## What This Project Demonstrates

| Domain | Implementation |
| :--- | :--- |
| **Firmware architecture** | Non-blocking cooperative state machine, 50 ms deterministic control loop, decoupled 300 ms UI loop |
| **Real-time control** | Tuned PID with integral anti-windup driving a non-linear thermal plant with significant transport delay |
| **Peripheral drivers** | Register-level SPI driver for the MAX31865 RTD-to-digital converter; quadrature encoder decoding via hardware interrupts |
| **Power electronics** | Zero-crossing synchronised phase-angle dimmer for AC infrared emitters; H-bridge DC motor drive with current limiting |
| **Hardware design** | Custom mixed-signal PCB in KiCad — precision analog front-end, switching regulator, motor drive, and isolated mains control interface |
| **Safety engineering** | Hardware emergency stop, thermal throttling, driver fault detection, mains isolation |

---

## System Overview

EMARI automates solvent extraction of plant metabolites using infrared radiation as the heat source. The engineering problem is not chemistry — it is holding a precise thermal profile on a plant with large transport delay and variable thermal mass, while concurrently driving agitation motors, servicing a menu-driven interface, and monitoring for fault conditions, all on a single microcontroller with no operating system.

The system was developed as a BEIFI undergraduate research project at Instituto Politécnico Nacional (June 2024 – May 2025), and is now being re-engineered around a purpose-built control board.

---

## TMC-1 — Thermal & Motion Controller

> **Status: schematic complete, layout in progress. Not yet fabricated.**

The original prototype was built on prototyping hardware with off-the-shelf breakout modules. **TMC-1** is a custom four-layer PCB that consolidates the entire control stack onto a single board.

### Design goals

- Eliminate breakout-module wiring as a source of noise and intermittent faults
- Isolate the precision analog measurement path from motor and switching noise
- Keep all mains switching off the board, behind a certified external module
- Provide hardware fault reporting that the firmware can act on

### Block architecture

```
12V DC in ──┬─ protection (polyfuse, reverse-polarity MOSFET, TVS)
            │
            ├─ AP62250 buck ──► 5V ──┬─ LCD backpack
            │                        │
            │                        └─ AMS1117 LDO ──► 3V3 ──┬─ ESP32-S3-WROOM-1
            │                                                  ├─ MAX31865
            │                                                  └─ dimmer control
            │
            └─ 2× DRV8872 H-bridge ──► DC gearmotors
```

### Subsystems

**Temperature acquisition**
MAX31865 RTD-to-digital converter in 3-wire PT100 configuration, with a 430 Ω 0.1 % reference resistor. Includes a 10 MΩ cable-break detection resistor so a severed probe lead reads full-scale rather than producing undefined values — a safety requirement when the measurement gates a heating element.

**Motor drive**
Two DRV8872 H-bridges with per-channel current limiting set by a 0.33 Ω sense resistor (≈1 A trip). Each driver's open-drain `nFAULT` output is routed to a GPIO, so overcurrent, thermal shutdown, and undervoltage lockout are detectable in firmware rather than inferred after the fact.

**Encoder interface**
Quadrature encoder inputs level-shifted from 5 V to 3.3 V by resistive dividers, with RC filtering on each channel to reject commutation noise. Decoded by the ESP32-S3 PCNT peripheral in hardware.

**IR lamp control**
The board carries no mains. Phase-angle control is delegated to an external opto-isolated triac module; the PCB exposes only a low-voltage interface — control output plus a filtered zero-cross input for firmware synchronisation.

**HMI**
I²C LCD driven through a bidirectional MOSFET level shifter (5 V display, 3.3 V logic), plus a filtered rotary encoder input.

**Programming and debug**
Native USB-C with CC pull-downs and a shield RC network. A separate UART0 header is broken out so the hardware serial console remains available when USB enumeration fails.

### GPIO allocation

| Peripheral | Pins |
| :--- | :--- |
| MAX31865 SPI | IO12 SCLK · IO13 MOSI · IO14 MISO · IO15 CS · IO16 DRDY |
| Motor 1 | IO40/IO41 drive · IO42 fault · IO38/IO39 encoder |
| Motor 2 | IO17/IO18 drive · IO21 fault · IO47/IO48 encoder |
| IR dimmer | IO10/IO11 control · IO07 zero-cross |
| HMI | IO08 SDA · IO09 SCL · IO04/IO05/IO06 rotary encoder |
| USB / Debug | IO19/IO20 native USB · IO43/IO44 UART0 |
| Reserved | IO00/IO03/IO45/IO46 strapping · IO35–IO37 Octal PSRAM · IO01/IO02 free (ADC1) |

---

## Firmware Architecture

Written in bare-metal C/C++ with no RTOS. Concurrency is achieved through cooperative scheduling and hardware interrupts rather than threads.

### Non-blocking state machine

Multiple physical subsystems run concurrently under timer polling (`millis()` / `micros()`):

- **Control loop — 50 ms.** Deterministic sampling interval governing PID evaluation, motor modulation, and cooling logic. Fixed period is a correctness requirement: the integral and derivative terms assume constant `dt`.
- **UI loop — 300 ms.** Decoupled from the control loop so display refresh cannot introduce jitter into the thermal regulation.
- **Hardware interrupts.** Encoder edge capture for RPM calculation, independent of loop timing.

### PID temperature control

Output = (K<sub>p</sub> · e) + (K<sub>i</sub> · ∫e dt) + (K<sub>d</sub> · de/dt)

Tuned to `Kp = 17.0`, `Ki = 0.5`, `Kd = 190` with active integral windup clamping. The high derivative gain compensates for thermal transport delay between the emitter and the sensed medium — the dominant difficulty in this plant.

### Safety systems

- **Hard-stop routine** — immediate cessation of all PWM output (motors, fans, heaters) on emergency stop, followed by a locked reboot sequence
- **Thermal throttling** — heating cut and exhaust ventilation forced to maximum if board temperature exceeds limits
- **Driver fault handling** — `nFAULT` assertion halts the affected motor; repeated faults are treated as a mechanical condition rather than retried indefinitely

---

## Operational Configurations

The chassis and firmware support two extraction methodologies. The state machine swaps thermal targets, motor scaling limits, and sensor averaging depending on the selected configuration.

| Configuration | CAD | Prototype |
| :--- | :---: | :---: |
| **Mode A — Continuous Reflux**<br>Boiling flask with vertical condenser column. High thermal mass; magnetic agitation via Motor 2 for continuous fluid movement. Optimised for exhaustive, large-volume extraction. | <img src="images/Reflux_CAD.JPG" width="230" alt="Reflux CAD"/> | <img src="images/Reflux_Physical.jpeg" width="230" alt="Reflux prototype"/> |
| **Mode B — Parallel Micro-Extraction**<br>Rotary carousel holding individual test tubes under direct infrared exposure. Low thermal mass, rapid-response thermal profile; rotary agitation via Motor 1 with RPM scaling limits. | <img src="images/Micro_extraction_CAD.JPG" width="230" alt="Micro-extraction CAD"/> | <img src="images/Micro_extraction_Physical.jpg" width="230" alt="Micro-extraction prototype"/> |

---

## Mechanical & Thermal

- High-density ceramic fiber insulation for thermodynamic efficiency
- Active dual 4-wire PWM fan cooling tied to on-board ambient sensing
- Modular chassis supporting both extraction configurations

---

## Repository Structure

```text
├── firmware/
│   ├── src/
│   │   ├── main.cpp            # State machine, HMI logic, interrupt handlers
│   │   ├── pid_control.cpp     # PID evaluation and thermal management
│   │   ├── motor_driver.cpp    # RPM calculation and dual-motor PWM
│   │   └── sensor_spi.cpp      # Register-level PT100 / LM35 drivers
│   └── include/
├── hardware/
│   └── tmc-1/
│       ├── TMC-1.kicad_pro     # KiCad project
│       ├── TMC-1.kicad_sch     # Schematic source
│       ├── TMC-1.kicad_pcb     # Board layout
│       ├── TMC-1_schematic.pdf # Rendered schematic
│       ├── bom/                # Bill of materials
│       └── docs/               # Design notes, pin map, datasheets
└── images/
```

---

## Roadmap

- [x] TMC-1 schematic capture and ERC clean
- [ ] PCB layout — ground partitioning between analog, digital, and motor domains
- [ ] Fabrication and assembly
- [ ] Board bring-up and validation against the prototype
- [ ] Migration of firmware to the custom board
- [ ] Rev B — switching regulator integration, thermal relocation of the reference resistor

---

## Author

**Noel Francisco Rodríguez**
B.S. Mechatronics Engineering, Instituto Politécnico Nacional
[github.com/NoelRodriguezz](https://github.com/NoelRodriguezz)
