# MSP430-Motor-Controller
Microcontroller system that monitors a "physical load" and drives a stepper motor based on safety thresholds simulated by varying voltages.

# Automated Load-Sensing Motor Controller

Project Overview:
This project is a microcontroller-based safety system built using the MSP430. It actively monitors a change in voltage and drives a stepper motor. If the applied weight exceeds safe limits, the system automatically halts the motor, triggers visual LED warnings, and logs a time-stamped error using an I2C Real-Time Clock.

Hardware & Components:
* Microcontroller: MSP430
* Actuator:Stepper Motor (513 steps/rev)
* Module: I2C Real-Time Clock (RTC)

Key Features & Technical Concepts:
* Hardware Interrupts: Utilizes Timer TB0 interrupts for precise motor stepping (Fast/Reverse vs. Slow/Precision) without blocking main code execution.
* Analog-to-Digital Conversion (ADC): Continuously samples sensor voltage to calculate physical load, implementing a three-tier safety logic system:
  * Safe (0–30 lbs): Green LED active, normal operation.
  * Warning (30–45 lbs): Yellow LED active, operation continues.
  * Unsafe (>45 lbs): Red LED active, motor halts, triggers logging sequence.
* I2C & UART Communication: Interfaces with an external RTC via I2C to pull current date/time data. Formats and transmits error logs via UART to a serial terminal (e.g., `Unsafe at 12:30:00 12/01/25`).

Notes about the Code:
The source code (`FinalDEMO.c`) is written entirely in C and demonstrates bare-metal microcontroller programming, register-level configuration, and custom ISR (Interrupt Service Routine) handling.
