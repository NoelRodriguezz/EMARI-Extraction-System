# EMARI: Infrared Radiation-Assisted Metabolite Extractor

A state-machine-driven, PID-controlled mechatronic prototype designed for precise, automated chemical extraction utilizing infrared-assisted radiation and dual-motor physical agitation.

---

## System Overview

To develop this prototype, a complete mechatronic design methodology was implemented, successfully bridging virtual CAD optimization with real-world physical deployment. 

The EMARI system integrates custom mechanical design, power electronics, and intelligent firmware to solve non-linear thermodynamic control challenges in metabolic extraction. The system maintains ultra-precise thermal profiles and safeguards hardware components under severe physical resource constraints.

---

## Operational Configurations

The mechanical chassis and control firmware are engineered to be modular, supporting two distinct extraction methodologies. The system dynamically adapts its non-blocking state machine and PID parameters based on the selected physical setup.

| Configuration | 3D CAD Design | Physical Prototype |
| :--- | :---: | :---: |
| **Mode A: Continuous Reflux (Magnetic Agitation)**<br><br>Utilizes a central boiling flask coupled with a vertical condenser column. Optimized for exhaustive, large-volume extraction.<br><br>*Firmware Profile:* Configured for high thermal mass; utilizes magnetic agitation (Motor 2) to maintain steady, continuous fluid movement. | <img src="images/Reflux_CAD.JPG" width="250" alt="Reflux CAD"/> | <img src="images/Reflux_Physical.jpeg" width="250" alt="Reflux Physical"/> |
| **Mode B: Parallel Micro-Extraction (Rotary Agitation)**<br><br>Utilizes a custom rotary carousel holding individual test tubes directly exposed to the infrared radiation.<br><br>*Firmware Profile:* Rapid-response thermal profile for low thermal mass; utilizes the rotary motor (Motor 1) with specific RPM scaling limits. | <img src="images/Micro_extraction_CAD.JPG" width="250" alt="Micro-Extraction CAD"/> | <img src="images/Micro_extraction_Physical.jpg" width="250" alt="Micro-Extraction Physical"/> |

> **Firmware Adaptability Note:** The ESP32 utilizes an internal state-machine to swap between the specific thermal targets, motor scaling limits, and sensor averaging logic required for the structural physics of Configuration A versus Configuration B.

---

## Hardware & Electromechanical Architecture

The prototype is built around a centralized control unit interfacing with industrial-grade sensing and high-torque mechanical actuation, supported by an active cooling ecosystem.

*   **Microcontroller:** ESP32-S3 (Dual-core Xtensa LX7, running at 240 MHz).
*   **Thermal Insulation:** High-density ceramic fiber insulation to optimize thermodynamic efficiency.
*   **Thermal Management (PCB):** Active dual 4-wire PWM fan cooling system directly tied to on-board ambient LM35 sensors to prevent dimmer/driver overheating. 
*   **Actuation:** Dual integrated DC/BLDC motors managing rotary mechanism speeds and magnetic agitation rates via PWM. IR heating is modulated via a zero-crossing synchronized dimmer circuit.
*   **Sensors:** Industrial PT100 (via MAX31865 SPI) and an array of LM35 temperature sensors for real-time thermal monitoring and hardware protection.
*   **Human-Machine Interface (HMI):** 20x4 I2C Liquid Crystal Display paired with a rotary encoder, active buzzer, and a hardwired emergency stop button.

---

## Software & Firmware Architecture

The firmware is written in bare-metal **C/C++** and engineered to handle concurrent processes without thread blocking, utilizing a highly responsive state machine.

### 1. Non-Blocking State Machine
Instead of a heavy RTOS, the firmware governs multiple physical subsystems concurrently utilizing precise timer polling (`millis()` and `micros()`) and hardware interrupts:
*   **Hardware Interrupts:** Real-time encoder reading for accurate motor RPM calculation.
*   **Control Loop:** A strictly timed 50ms sampling rate (`SAMPLE_TIME`) dictates the execution of temperature control, motor modulation, and active cooling logic.
*   **UI Loop:** A decoupled 300ms refresh rate for the HMI to prevent display flickering and ensure fluid menu navigation.

### 2. PID Temperature Control
EMARI deploys a highly tuned Proportional-Integral-Derivative (PID) algorithm to handle thermodynamic delays and modulate the infrared heating elements. 
*   **Control Equation:** The system calculates real-time heating power based on the error $e(t)$ between the target temperature and the fused sensor data.
    $$Output = (K_p \cdot e) + (K_i \cdot \int e) + (K_d \cdot \frac{de}{dt})$$
*   The system actively constraints the integral windup and utilizes $K_p = 17.0$, $K_i = 0.5$, and $K_d = 190$ to maintain absolute stability during extraction.

### 3. Active Safety Systems
*   **Hard-Stop Routine:** Immediate halting of all PWM signals (Motors, Fans, Heaters) upon physical emergency stop activation, followed by a locked reboot sequence.
*   **PCB Thermal Throttling:** If the internal control board exceeds safe parameters, heating is automatically cut off and maximum exhaust ventilation is engaged.

---

##  Repository Structure

```text
├── firmware/
│   ├── src/
│   │   ├── main.cpp            # Core state machine, HMI logic, and interrupts
│   │   ├── pid_control.cpp     # PID temperature calculation and hardware thermal management
│   │   ├── motor_driver.cpp    # RPM calculation and PWM driver for dual-motor control
│   │   └── sensor_spi.cpp      # Register-level PT100/LM35 driver code
│   └── include/                # Firmware header files
├── hardware/
│   ├── schematics/             # Circuit diagrams and power stage layouts
│   └── docs/                   # Material datasheets (Ceramic Fiber, Sensor specs)
└── images/                     # System schematics, photos, and performance plots
    ├── Micro_extraction_CAD.JPG
    ├── Micro_extraction_Physical.jpg
    ├── Reflux_CAD.JPG
    └── Reflux_Physical.jpeg
