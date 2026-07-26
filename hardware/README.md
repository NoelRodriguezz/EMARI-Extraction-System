# Hardware Architecture & Electronics

This directory contains the electrical schematics, PCB layout files, and component datasheets for the **Infrared Radiation-Assisted Metabolite Extractor (EMARI)**.

The system is controlled by an **ESP32-S3** microcontroller, which handles real-time PID calculations, sensor polling, motor PWM generation, and the active cooling subsystem.

## Component Subsystems Overview

This table groups the hardware by physical subsystems, making it easier to understand the module-level architecture and how the components interact with the microcontroller.

| Subsystem Group | Specific Component | ESP32-S3 Pins | Function / Role |
| :--- | :--- | :--- | :--- |
| **Motor Actuation** | Motor 1 (Rotary) | 36, 35, 7 | PWM Control, Ground/Enable, Encoder Interrupt |
| **Motor Actuation** | Motor 2 (Magnetic) | 0, 45, 17 | PWM Control, Ground/Enable, Encoder Interrupt |
| **Active Cooling** | Enclosure Fan 1 | 39, 40 | PWM Speed Control, Tachometer Feedback |
| **Active Cooling** | Enclosure Fan 2 | 41, 42 | PWM Speed Control, Tachometer Feedback |
| **Infrared Heating**| Triac Dimmer Modules | 48, 21, 38 | Firing Control Signals (Dimmers 1, 2, and 3) |
| **Infrared Heating**| Sync Circuit | 47 | AC Mains Zero-Crossing Synchronization |
| **Thermal Sensors** | PT100 (MAX31865) | 10, 11, 12, 13 | SPI Bus Communication (CS, MOSI, SCK, MISO) |
| **Thermal Sensors** | LM35 Temp Array | 15, 16, 18 | Analog Voltage (Chamber 1, Chamber 2, Main PCB) |
| **User Interface** | Rotary Encoder | 4, 5, 6 | Navigation Signals (Clock, Data, and Switch) |
| **User Interface** | 20x4 I2C LCD | 46, 9 | I2C Bus Communication (SDA, SCL) |
| **User Interface** | Active Buzzer | 8 | System Audio Feedback and Thermal Alarms |
| **Safety System** | Emergency Stop | 1 | Hardware Interrupt for total system halt |

---

## Core Electronic Modules

The following breakout boards and integrated circuits serve as the interface layer between the ESP32-S3 logic and the physical extraction environment:

*   **AC Triac Dimmer Modules:** Used to safely modulate the AC mains voltage powering the infrared radiation heaters. These modules rely on a shared zero-crossing synchronization signal to time the triac firing, allowing the PID controller to output smooth, flicker-free thermal adjustments.
*   **Motor Driver Power Stages:** Intermediary power modules that isolate the microcontroller from electrical noise. They translate the 3.3V logic and PWM signals from the ESP32 into the high-current power required to drive the rotary and magnetic agitation motors.
*   **MAX31865 RTD-to-Digital Converter:** A specialized amplifier and digital converter module required to read the minute resistance changes of the PT100 platinum sensor. It processes the analog signal and transmits highly accurate thermal data to the ESP32 via the SPI bus.
*   **LM35 Precision Analog Sensors:** Integrated-circuit temperature devices that output an analog voltage directly proportional to the ambient temperature. These are strategically placed to monitor internal chamber gradients and protect the main control PCB from overheating.
*   **I2C LCD Backpack:** A serial interface module soldered directly to the 20x4 LCD. It condenses the parallel display connections down to a standard 2-wire I2C bus (SDA and SCL), drastically freeing up GPIO pins for other mechanical tasks.
*   **Rotary Encoder Module:** An electro-mechanical device serving as the primary Human-Machine Interface. It converts the angular rotation of the user's knob into digital quadrature pulses for menu navigation and includes a built-in push-button switch for making selections.

---

## Microcontroller Pin Mapping (ESP32-S3)

The following table details the physical pin connections sequentially. Ensure all SPI and I2C lines have appropriate pull-up resistors as defined in the schematics.

| Pin | Subsystem | Task / Description | Connected To |
| :---: | :--- | :--- | :--- |
| **0** | Motors | Motor 2 (Magnetic) PWM Signal | Magnetic Motor Driver |
| **1** | Safety | Hardwired Emergency Stop | Physical E-Stop Button |
| **4** | HMI | Rotary Encoder Clock Signal | Encoder (CLK) |
| **5** | HMI | Rotary Encoder Data Signal | Encoder (DT) |
| **6** | HMI | Rotary Encoder Switch Signal | Encoder (SW) |
| **7** | Motors | Motor 1 Encoder Pulse Interrupt | Rotary Motor Encoder |
| **8** | HMI | System Alarms & Audio Feedback | Active Buzzer |
| **9** | HMI (I2C) | I2C Serial Clock (SCL) | 20x4 LCD Display |
| **10** | Sensors (SPI) | Chip Select (CS) | MAX31865 Amplifier (PT100) |
| **11** | Sensors (SPI) | Master Out Slave In (MOSI) | MAX31865 Amplifier (PT100) |
| **12** | Sensors (SPI) | Serial Clock (SCK) | MAX31865 Amplifier (PT100) |
| **13** | Sensors (SPI) | Master In Slave Out (MISO) | MAX31865 Amplifier (PT100) |
| **15** | Sensors (ADC) | Analog reading for Chamber 1 | LM35 Sensor 1 |
| **16** | Sensors (ADC) | Analog reading for Chamber 2 | LM35 Sensor 2 |
| **17** | Motors | Motor 2 Encoder Pulse Interrupt | Magnetic Motor Encoder |
| **18** | Sensors (ADC) | Analog reading for PCB ambient temp | LM35 Sensor (Control Board) |
| **21** | Heaters | Dimmer 2 Control Signal | IR Source Dimmer Circuit 2 |
| **35** | Motors | Motor 1 (Rotary) Ground/Enable | Rotary Motor Driver |
| **36** | Motors | Motor 1 (Rotary) PWM Signal | Rotary Motor Driver |
| **38** | Heaters | Dimmer 3 Control Signal | IR Source Dimmer Circuit 3 |
| **39** | Cooling | Fan 1 PWM Control | 4-Wire Exhaust Fan 1 |
| **40** | Cooling | Fan 1 Tachometer Input | 4-Wire Exhaust Fan 1 |
| **41** | Cooling | Fan 2 PWM Control | 4-Wire Exhaust Fan 2 |
| **42** | Cooling | Fan 2 Tachometer Input | 4-Wire Exhaust Fan 2 |
| **45** | Motors | Motor 2 (Magnetic) Ground/Enable | Magnetic Motor Driver |
| **46** | HMI (I2C) | I2C Serial Data (SDA) | 20x4 LCD Display |
| **47** | Heaters | AC Zero-Crossing Synchronization | AC Dimmer Sync Circuit |
| **48** | Heaters | Dimmer 1 Control Signal | IR Source Dimmer Circuit 1 |

## Directory Contents

*   `/schematics/`: Contains the PDF circuit diagrams and Gerber files for the custom PCB power stages.
*   `/docs/`: Contains engineering datasheets for the critical components used in this build:
    *   ESP32-S3 WROOM Technical Reference
    *   MAX31865 RTD-to-Digital Converter
    *   LM35 Precision Centigrade Temperature Sensors
    *   Ceramic Fiber Thermal Insulation Specifications

## Power and Safety Notes

*   **Logic Level:** The ESP32-S3 operates on **3.3V logic**. Ensure proper level shifting is applied if interfacing with 5V components.
*   **Thermal Safety:** The PCB LM35 sensor (Pin 18) must be physically mounted near the motor drivers and dimmer triacs to accurately trigger the active cooling exhaust fans (Pins 39-42) and prevent thermal runaway.
*   **AC Mains:** The dimmer circuits interface directly with high voltage AC to power the IR sources. Always disconnect main power before opening the control enclosure.
