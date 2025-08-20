# Robotic Arm Controller - VHDL Implementation

## Overview
This project implements a 4-degree-of-freedom robotic arm controller using VHDL. The design features a ring counter that sequentially activates motor enable signals in either clockwise or counter-clockwise direction based on user input.

## Key Features

- **Clock**: 50MHz internal crystal (Pin P23)
- **Control Inputs**:
  - Reset button (synchronized)
  - Step button (with debouncing)
  - Direction switch
- **Output**: One-hot encoded motor enable signals (4 motors)

## Architecture

### Main Components
1. **Debouncer Circuit**: Filters button presses to prevent multiple triggering
2. **Ring Counter**: Sequential circuit that rotates a single '1' through its outputs
3. **Direction Control**: Determines shift direction (left/right)

### Operation
- **Initial State**: motor_enable = "0001" (Joint 0 active)
- **Reset**: Returns to initial state
- **Step Advance**: On each debounced button press, shifts the active motor
- **Direction Control**:
  - dir_sw = '0': Rotate right (Joint 0 → 3 → 2 → 1 → 0)
  - dir_sw = '1': Rotate left (Joint 0 → 1 → 2 → 3 → 0)

## RTL Viewer Description
The RTL schematic would show:
- A 4-bit register (r_q) implementing the ring counter
- A debouncing circuit with edge detection
- Multiplexers for directional control
- Clock synchronization elements
- Output directly driving the motor enable signals

## Usage
1. Set direction using dir_sw
2. Press step_btn to advance to next motor
3. Use rst_button to return to initial position

This design provides precise control over motor activation sequence with clean signal transitions ensured through debouncing and synchronous design practices.
