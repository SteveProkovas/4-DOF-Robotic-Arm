# 4-DOF Robotic Arm Prototype — Educational Edition v1.0

**Project summary (Executive summary)**
This repository documents the design and implementation of a professional-quality prototype 4-DOF robotic arm using 4 hobby servos, with the **DSD-i1** development board (STM32F1 + FPGA) as the main controller for PWM generation and system coordination. The first phase implements open-loop position control (fast prototyping), and the design, code and hardware layout are structured for a later engineering upgrade to closed-loop actuators with encoders and FPGA acceleration.

> References: consult the DSD-i1 pinout for exact header mappings and the laboratory guide for FPGA / STM32 design flow. &#x20;

---

## Table of Contents

1. Goals & Deliverables
2. System architecture (high level)
3. Professional BOM (recommendations)
4. Required tools & software
5. Quick start (5-minute executive flow)
6. Wiring & power (safe, production-minded)
7. CubeMX / Timer configuration (validated numeric)
8. Firmware structure & critical snippets
9. Calibration, validation & KPIs
10. Upgrade roadmap (encoders + FPGA)
11. Deliverables, license & contribution guidelines
12. Next steps (what I will produce for you)

---

## 1. Goals & Deliverables

**Primary goals**

* Deliver a robust, demonstrable 4-DOF arm prototype suitable for demos and portfolio use.
* Provide engineering-grade documentation, reproducible build flow, safety controls and test procedures.
* Implement Phase-1: open-loop control using 4 hobby servos and STM32 PWM.
* Provide a clear migration path to Phase-2: closed-loop actuation using encoders, motor drivers and FPGA modules.

**Phase-1 deliverables**

* Wiring diagrams and precise pin mapping for DSD-i1.&#x20;
* CubeMX project template (TIMx PWM configuration) and ready-to-flash HAL firmware.
* PCA9685 I²C driver example (optional approach for fast scaling).
* Test scripts, calibration utilities and performance logging templates.

---

## 2. System architecture (concise)

* **High-level controller:** DSD-i1 (STM32F103) — PWM generation, CLI/telemetry, optional I²C to PWM expander.&#x20;
* **Actuation (Phase-1):** 4 × hobby servos (open-loop).
* **Power:** external 5 V supply for servos (fused + kill switch). DSD-i1 powered independently (USB / on-board regulator).
* **Safety & monitoring:** fuse, kill switch, shared GND, current & temperature monitoring (recommended).

---

## 3. Professional BOM (recommended)

**Prototype (Phase-1)**

* DSD-i1 development board (STM32F103 + Cyclone IV FPGA).&#x20;
* 4 × hobby servos — prefer metal-gear, choose torque rating to match arm geometry (e.g., MG90S for small loads; MG996R/metal gear for higher torque).
* 5 V PSU, 6 A (or LiPo + high-current BEC capable of stall currents).
* Fuse 6 A (fast blow), inline kill-switch (SPST).
* PCA9685 16-channel PWM driver (Adafruit / compatible) — optional (recommended for rapid development).
* Common-ground wiring, JST/servo connectors, heat-shrink, PDB (power distribution board).
* Mechanical parts: 3D-printed brackets or laser-cut plates, bearings, limit stops.

**Upgrade (Phase-2)**

* Incremental encoders (100–1024 CPR) or Dynamixel smart actuators (for integrated feedback).
* Motor drivers (Roboclaw / ODrive / custom H-bridge) as appropriate.
* FPGA toolchain (Quartus), VHDL testbenches, CAN/SPI hardware for MCU↔FPGA comms.

---

## 4. Required tools & software

* STM32CubeIDE / STM32CubeMX (generate config + flash)
* ST-Link / USB interface for flashing the DSD-i1
* Python 3.8+ (scripts for calibration & logs)
* Multimeter, current clamp or bench PSU with current readout
* Oscilloscope / logic analyzer (recommended for PWM validation)
* Optional: Adafruit PCA9685 library examples (for PCA9685 route)

---

## 5. Quick start (5-minute executive flow)

1. Review the DSD-i1 pinout and lab guide. &#x20;
2. Wire the PSUs: PSU +5 V → fuse → kill-switch → servo V+. Do **not** power servos from the DSD-i1.
3. Connect PSU GND to the DSD-i1 GND (single common ground).&#x20;
4. If using PCA9685: wire SDA/SCL to STM32 I²C pins and power PCA9685 from servo PSU.
5. Flash the CubeMX HAL project (TIMx PWM config) to the DSD-i1.
6. Send initial neutral pulses (1.5 ms) to each servo; verify mechanical limits and safe motion.

---

## 6. Wiring & power — safe, production-minded (textual schematic)

**Power distribution (recommended)**

```
PSU +5V  ──→ [Fuse 6A] ──→ [Kill switch] ──→ Servo V+
PSU GND ───────────────────────────────────→ common GND bus ──→ DSD-i1 GND
```

**Signal wiring options**

* **Direct PWM (Option A)** — STM32 TIMx\_CH1..CH4 → Servo SIGNALs (use pins mapped to timers on DSD-i1). Ensure the STM32 pin mapping uses timer channels with the CubeMX config.&#x20;
* **PCA9685 I²C (Option B, recommended for speed & scaling)** — PCA9685 V+ → servo PSU; PCA9685 GND → common GND; PCA9685 SDA/SCL → STM32 I2C pins; PCA9685 OUT0..3 → servo SIGNALs.

**Signal level note:** DSD-i1 I/O is 3.3 V TTL. Most hobby servos accept 3.3 V control signals; verify servo input tolerance. If a servo strictly requires 5 V logic, add a level shifter.

---

## 7. CubeMX / Timer configuration (validated numeric)

**Assumptions:** STM32 timer clock = 72 MHz (typical for STM32F1).

**Target:** 50 Hz PWM (20 ms period), fine resolution (\~1 µs per tick).
**Recommended timer settings**

* Prescaler (PSC) = **71** → timer tick = 72 MHz / (PSC + 1) = 1 MHz → 1 tick = 1 µs
* Auto-reload (ARR) = **19,999** → period = 20,000 ticks = 20 ms → 50 Hz
* Compare (CCR) mapping: 1.0 ms → 1000, 1.5 ms → 1500, 2.0 ms → 2000

**CubeMX steps (summary)**

1. Set SystemClock = 72 MHz.
2. Enable TIMx (e.g., TIM2) and configure CH1..CH4 as PWM Generation (Mode 1).
3. Set PSC = 71, ARR = 19999. Generate project and open in STM32CubeIDE.
4. Implement `set_servo_ms()` wrapper to map ms → CCR.

---

## 8. Firmware structure & critical snippets

**Suggested repository layout**

```
/firmware
  /Core, /Drivers (CubeMX generated)
  /Src
    main.c
    pwm_control.c       // PWM abstraction
    i2c_pca9685.c       // optional PCA9685 driver
    cli.c               // serial command interface
  /Inc
    pwm_control.h
    config.h
```

**Critical servo setter (HAL)**

```c
// PSC = 71, ARR = 19999 -> 1 tick = 1 us
void set_servo_ms(TIM_HandleTypeDef *htim, uint32_t channel, float ms) {
    uint32_t compare = (uint32_t)(ms * 1000.0f); // 1.5 ms => 1500
    __HAL_TIM_SET_COMPARE(htim, channel, compare);
}
```

**PCA9685 minimal write (HAL I2C)**

```c
// Using HAL I2C, PCA9685 default addr 0x40 (7-bit)
void pca9685_set_pwm(I2C_HandleTypeDef *hi2c, uint8_t channel, uint16_t on, uint16_t off) {
  uint8_t buf[5];
  buf[0] = LED0_ON_L + 4*channel;
  buf[1] = on & 0xFF;
  buf[2] = (on >> 8) & 0xFF;
  buf[3] = off & 0xFF;
  buf[4] = (off >> 8) & 0xFF;
  HAL_I2C_Master_Transmit(hi2c, PCA_ADDR, buf, 5, HAL_MAX_DELAY);
}
```

**Command interface (recommended)**

* Implement a simple serial CLI: `SERVO <id> <ms>`, `CALIB <id>`, `STATUS`, `TESTCYCLE`. This is essential for demonstrations and calibration.

---

## 9. Calibration, validation & KPIs (engineering tests)

**Calibration procedure**

1. Neutral: set 1.5 ms → verify mechanical center.
2. Sweep: slowly sweep 1.0 ms → 2.0 ms to detect mechanical endpoints. Record safe min/max (e.g., 1.05–1.95 ms).
3. Map ms ↔ joint angle: sample a few points and fit linear mapping; save per-joint JSON calibration.

**Validation tests & KPIs**

* **Repeatability:** (with encoders later) target < 1° RMSE.
* **Power:** measure idle, nominal motion and stall currents; PSU should maintain voltage within 5% under load.
* **Thermal:** servo temperature after 10 min cycling < 60 °C (target).
* **Latency:** command to movement measured via logic analyzer (documented).

**Acceptance criteria example**

* No fuse trips in normal operation; kill switch cuts servo power immediately.
* Repeatability within mechanical limits; no mechanical interference.
* Smooth motion without jitter (verify common GND, clean PWM).

---

## 10. Upgrade roadmap (Phase-2: encoders + FPGA acceleration)

**Short term**

* Add encoder to one joint; validate quadrature decoding and closed-loop PID on STM32.
* Implement a VHDL quadrature decoder module and testbench (ModelSim).&#x20;

**Medium term**

* Offload inner control loop (fast PID) to FPGA for ultra low latency; STM32 remains trajectory & high-level planner.
* Use robust MCU↔FPGA communication (SPI / CAN) with heartbeat and telemetry.
* Integrate safety interlocks in FPGA (watchdog, current limit monitor).

**Deliverables for Phase-2**

* VHDL `quad_decoder` + testbench.
* FPGA ↔ STM32 comms protocol spec.
* Closed-loop tuning report (PID gains, bandwidth tests).

---

## 11. Deliverables, license & contributing

**Repository contents (recommended)**

* `/hardware` — wiring diagrams (PDF/PNG), BOM.csv
* `/firmware` — CubeMX project, HAL code, PCA9685 driver, example CLI
* `/tests` — calibration scripts, log parsers, sample CSV results
* `/docs` — this README, safety report template, upgrade roadmap

**License**
Use a permissive license for educational & portfolio use.


**Status & Contact**
Prepared by: 
`[Steve Stavros Prokovas / Hellenic Open University Laboratory of Mobile and Diffuse Computing, Quality and Surrounding Intelligence]`
Version: v1.0 — Educational Prototype Release.
