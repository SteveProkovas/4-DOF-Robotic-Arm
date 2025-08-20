# Robotic Arm Controller - VHDL Implementation

## Overview
This project implements a 4-degree-of-freedom robotic arm controller using VHDL. The design features a state machine that controls four motors with position tracking, limit switch protection, and PWM-based velocity control.

## Block Diagram
```
┌─────────────────┐    ┌─────────────────┐    ┌──────────────────┐
│   Input Signals │    │   Control Unit  │    │  Output Signals  │
│                 │    │                 │    │                  │
│ • clk_50mhz     │───▶ • Sync Reset     ───▶│ • motor_enable   │
│ • rst_button    │    │ • Debouncer     │    │ • pwm_outputs    │
│ • step_btn      │    │ • Position Cnt  │    │ • position_out   │
│ • dir_sw        │    │ • State Machine │    │ • debug_led      │
│ • limit_switches│    │ • PWM Generator │    │                  │
└─────────────────┘    └─────────────────┘    └──────────────────┘
```

## Key Features

### 1. Input Processing
- **Clock**: 50MHz system clock
- **Synchronized Reset**: Metastability-protected reset signal
- **Debounced Step Input**: Noise-filtered button input
- **Direction Control**: Switch-based direction selection
- **Limit Switches**: Physical end-stop protection

### 2. Core Functionality
- **Position Counter**: 16-bit up/down counter tracking arm position
- **State Machine**: One-hot encoded controller for 4 motors
- **Limit Protection**: Prevents movement beyond mechanical limits
- **PWM Control**: Variable speed control with fixed 50% duty cycle

### 3. Output Signals
- **Motor Enable**: One-hot encoded enable signals (4 bits)
- **PWM Outputs**: Velocity control signals (4 bits)
- **Position Output**: 16-bit position feedback
- **Debug LEDs**: Visual status indicators (8 bits)

## Operation Modes

### Normal Operation
1. Direction switch sets rotation direction (CW/CCW)
2. Step button advances the state machine
3. Motors activate in sequence based on direction
4. Position counter increments/decrements accordingly

### Safety Features
- Limit switch detection prevents over-travel
- Reset returns system to initial state (Motor 0 active)
- Debouncing prevents false step detection

## Signal Description

### Inputs
- `clk_50mhz`: 50MHz master clock
- `rst_button`: Synchronized reset signal
- `step_btn`: Step advance button (debounced)
- `dir_sw`: Direction control (0=CCW, 1=CW)
- `limit_switches`: Physical limit switches (4 bits)

### Outputs
- `motor_enable`: Motor enable signals (one-hot)
- `pwm_outputs`: PWM velocity control signals
- `position_out`: Current position value
- `debug_led`: Diagnostic LEDs

## Implementation Details

The design uses several key processes:
1. **Synchronized Reset**: Creates stable reset signal
2. **Input Debouncing**: Filters mechanical switch noise
3. **Position Counting**: Tracks absolute position
4. **State Machine**: Controls motor sequencing
5. **PWM Generation**: Creates velocity control signals

The state machine implements circular shifting with direction control and limit protection, ensuring safe operation throughout the mechanical range.

## Applications
- Robotic arm control
- CNC machine control
- Stepper motor systems
- Precision positioning systems

This implementation provides a robust foundation for robotic control systems with safety features and position tracking capabilities.
