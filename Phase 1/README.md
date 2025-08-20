# 4-DOF Robot Arm — FSM Controller (D-Flip-Flop Implementation)

## Overview

This repository defines a synthesizable D-Flip-Flop (D-FF) based finite-state machine (FSM) to control a 4-DOF robotic arm for a **pick-and-place** sequence. The FSM is designed as a Moore machine (outputs depend only on current state) and is ready for synthesis on FPGA/ASIC as a control layer that sends pose commands to a motion controller and handshakes with it.

**High-level sequence (one cycle):**
`IDLE → PRE_PICK → MOVE_PICK → CLOSE_GRIPPER → LIFT → MOVE_PLACE → LOWER_PLACE → OPEN_GRIPPER → IDLE`

---

## Features

* Moore outputs: `pose[2:0]` (pose selector), `gripper` (0=open, 1=close), `busy`
* 3 D-FFs state register: simple and synthesizable
* Clear handshake inputs: `start`, `motion_done`, `grip_ok`
* Exhaustive next-state truth table (full 64 rows)
* DOT graph (Graphviz) for direct rendering of the state diagram
* Minimised next-state expressions for D inputs (D\_A, D\_B) and explicit SOP for D\_C

---

## State encoding (A B C)

We use 3 state bits: A (MSB), B, C (LSB) stored in three D-FFs.

| State name   | Encoding (A B C) | Description                    |
| ------------ | ---------------: | ------------------------------ |
| IDLE         |              000 | Waiting for `start`            |
| PRE\_PICK    |              001 | Move to pre-pick pose          |
| MOVE\_PICK   |              010 | Lower to pick pose             |
| CLOSE\_GRIP  |              011 | Close gripper (wait `grip_ok`) |
| LIFT         |              100 | Lift object                    |
| MOVE\_PLACE  |              101 | Move to place pose             |
| LOWER\_PLACE |              110 | Lower to place                 |
| OPEN\_GRIP   |              111 | Open gripper and finish        |

---

## Inputs and outputs

* **Inputs**:

  * `start` — external command to start a pick-and-place sequence (from higher-level planner)
  * `motion_done` — motion controller handshakes when it finishes moving to the requested `pose`
  * `grip_ok` — gripper sensor confirming it successfully gripped the object

* **Outputs (Moore)**:

  * `pose[2:0]` — pose selector sent to the motion controller (one pose per state)
  * `gripper` — 1 for close, 0 for open (closed only in CLOSE state)
  * `busy` — 1 while FSM is active (not IDLE)

---

## Compact next-state transitions (human readable)

```
IDLE (000)       -- start=1        --> PRE_PICK (001)
PRE_PICK (001)   -- motion_done=1  --> MOVE_PICK (010)
MOVE_PICK (010)  -- motion_done=1  --> CLOSE_GRIP (011)
CLOSE_GRIP (011) -- grip_ok=1      --> LIFT (100)
LIFT (100)       -- motion_done=1  --> MOVE_PLACE (101)
MOVE_PLACE (101) -- motion_done=1  --> LOWER_PLACE (110)
LOWER_PLACE (110)-- motion_done=1  --> OPEN_GRIP (111)
OPEN_GRIP (111)  -- motion_done=1  --> IDLE (000)
```

When the trigger for the transition is *not* asserted, the FSM stays in the same state (i.e., each state waits until its handshake/trigger becomes true).

---

## Graphviz DOT (state diagram)

Save the block below as `fsm.dot` and render with Graphviz:

```
digraph FSM {
  rankdir=LR;
  node [shape = circle];
  IDLE [label="IDLE\n000\nbusy=0"];
  PRE_PICK [label="PRE_PICK\n001"];
  MOVE_PICK [label="MOVE_PICK\n010"];
  CLOSE_GRIP [label="CLOSE_GRIP\n011\ngripper=1"];
  LIFT [label="LIFT\n100"];
  MOVE_PLACE [label="MOVE_PLACE\n101"];
  LOWER_PLACE [label="LOWER_PLACE\n110"];
  OPEN_GRIP [label="OPEN_GRIP\n111"];

  IDLE -> PRE_PICK [label="start=1"];
  IDLE -> IDLE [label="start=0"];

  PRE_PICK -> MOVE_PICK [label="motion_done=1"];
  PRE_PICK -> PRE_PICK [label="motion_done=0"];

  MOVE_PICK -> CLOSE_GRIP [label="motion_done=1"];
  MOVE_PICK -> MOVE_PICK [label="motion_done=0"];

  CLOSE_GRIP -> LIFT [label="grip_ok=1"];
  CLOSE_GRIP -> CLOSE_GRIP [label="grip_ok=0"];

  LIFT -> MOVE_PLACE [label="motion_done=1"];
  LIFT -> LIFT [label="motion_done=0"];

  MOVE_PLACE -> LOWER_PLACE [label="motion_done=1"];
  MOVE_PLACE -> MOVE_PLACE [label="motion_done=0"];

  LOWER_PLACE -> OPEN_GRIP [label="motion_done=1"];
  LOWER_PLACE -> LOWER_PLACE [label="motion_done=0"];

  OPEN_GRIP -> IDLE [label="motion_done=1"];
  OPEN_GRIP -> OPEN_GRIP [label="motion_done=0"];
}
```

---

## Complete next-state truth table (exhaustive)

**Columns:** A B C | start motion\_done grip\_ok || A+ B+ C+ | next\_state | pose | gripper | busy

> There are 64 rows (8 current-state combinations × 8 input combinations).
> Below is the full, exhaustive table (Markdown). It is the precise backbone for both deriving D-input logic and for writing the testbench.

```
| A | B | C | start | motion_done | grip_ok || A+ | B+ | C+ | next_state | pose | gripper | busy |
|---:|---:|---:|:-----:|:-----------:|:-----:||:--:|:--:|:--:|:----------:|:----:|:-------:|:----:|
| 0 | 0 | 0 |   0   |     0      |  0  ||  0 |  0 |  0 | IDLE       | 000 |    0    |  0   |
| 0 | 0 | 0 |   0   |     0      |  1  ||  0 |  0 |  0 | IDLE       | 000 |    0    |  0   |
| 0 | 0 | 0 |   0   |     1      |  0  ||  0 |  0 |  0 | IDLE       | 000 |    0    |  0   |
| 0 | 0 | 0 |   0   |     1      |  1  ||  0 |  0 |  0 | IDLE       | 000 |    0    |  0   |
| 0 | 0 | 0 |   1   |     0      |  0  ||  0 |  0 |  1 | PRE_PICK   | 000 |    0    |  0   |
| 0 | 0 | 0 |   1   |     0      |  1  ||  0 |  0 |  1 | PRE_PICK   | 000 |    0    |  0   |
| 0 | 0 | 0 |   1   |     1      |  0  ||  0 |  0 |  1 | PRE_PICK   | 000 |    0    |  0   |
| 0 | 0 | 0 |   1   |     1      |  1  ||  0 |  0 |  1 | PRE_PICK   | 000 |    0    |  0   |
| 0 | 0 | 1 |   0   |     0      |  0  ||  0 |  0 |  1 | PRE_PICK   | 001 |    0    |  1   |
| 0 | 0 | 1 |   0   |     0      |  1  ||  0 |  0 |  1 | PRE_PICK   | 001 |    0    |  1   |
| 0 | 0 | 1 |   0   |     1      |  0  ||  0 |  1 |  0 | MOVE_PICK  | 001 |    0    |  1   |
| 0 | 0 | 1 |   0   |     1      |  1  ||  0 |  1 |  0 | MOVE_PICK  | 001 |    0    |  1   |
| 0 | 0 | 1 |   1   |     0      |  0  ||  0 |  0 |  1 | PRE_PICK   | 001 |    0    |  1   |
| 0 | 0 | 1 |   1   |     0      |  1  ||  0 |  0 |  1 | PRE_PICK   | 001 |    0    |  1   |
| 0 | 0 | 1 |   1   |     1      |  0  ||  0 |  1 |  0 | MOVE_PICK  | 001 |    0    |  1   |
| 0 | 0 | 1 |   1   |     1      |  1  ||  0 |  1 |  0 | MOVE_PICK  | 001 |    0    |  1   |
| 0 | 1 | 0 |   0   |     0      |  0  ||  0 |  1 |  0 | MOVE_PICK  | 010 |    0    |  1   |
| 0 | 1 | 0 |   0   |     0      |  1  ||  0 |  1 |  0 | MOVE_PICK  | 010 |    0    |  1   |
| 0 | 1 | 0 |   0   |     1      |  0  || 0  |  1 |  1 | CLOSE_GRIP | 010 |    0    |  1   |
| 0 | 1 | 0 |   0   |     1      |  1  || 0  |  1 |  1 | CLOSE_GRIP | 010 |    0    |  1   |
| 0 | 1 | 0 |   1   |     0      |  0  || 0  |  1 |  0 | MOVE_PICK  | 010 |    0    |  1   |
| 0 | 1 | 0 |   1   |     0      |  1  || 0  |  1 |  0 | MOVE_PICK  | 010 |    0    |  1   |
| 0 | 1 | 0 |   1   |     1      |  0  || 0  |  1 |  1 | CLOSE_GRIP | 010 |    0    |  1   |
| 0 | 1 | 0 |   1   |     1      |  1  || 0  |  1 |  1 | CLOSE_GRIP | 010 |    0    |  1   |
| 0 | 1 | 1 |   0   |     0      |  0  || 0  |  1 |  1 | CLOSE_GRIP | 011 |    1    |  1   |
| 0 | 1 | 1 |   0   |     0      |  1  || 0  |  1 |  1 | CLOSE_GRIP | 011 |    1    |  1   |
| 0 | 1 | 1 |   0   |     1      |  0  || 1  |  0 |  0 | LIFT       | 011 |    1    |  1   |
| 0 | 1 | 1 |   0   |     1      |  1  || 1  |  0 |  0 | LIFT       | 011 |    1    |  1   |
| 0 | 1 | 1 |   1   |     0      |  0  || 0  |  1 |  1 | CLOSE_GRIP | 011 |    1    |  1   |
| 0 | 1 | 1 |   1   |     0      |  1  || 0  |  1 |  1 | CLOSE_GRIP | 011 |    1    |  1   |
| 0 | 1 | 1 |   1   |     1      |  0  || 1  |  0 |  0 | LIFT       | 011 |    1    |  1   |
| 0 | 1 | 1 |   1   |     1      |  1  || 1  |  0 |  0 | LIFT       | 011 |    1    |  1   |
| 1 | 0 | 0 |   0   |     0      |  0  || 1  |  0 |  0 | LIFT       | 100 |    0    |  1   |
| 1 | 0 | 0 |   0   |     0      |  1  || 1  |  0 |  0 | LIFT       | 100 |    0    |  1   |
| 1 | 0 | 0 |   0   |     1      |  0  || 1  |  0 |  1 | MOVE_PLACE | 100 |    0    |  1   |
| 1 | 0 | 0 |   0   |     1      |  1  || 1  |  0 |  1 | MOVE_PLACE | 100 |    0    |  1   |
| 1 | 0 | 0 |   1   |     0      |  0  || 1  |  0 |  0 | LIFT       | 100 |    0    |  1   |
| 1 | 0 | 0 |   1   |     0      |  1  || 1  |  0 |  0 | LIFT       | 100 |    0    |  1   |
| 1 | 0 | 0 |   1   |     1      |  0  || 1  |  0 |  1 | MOVE_PLACE | 100 |    0    |  1   |
| 1 | 0 | 0 |   1   |     1      |  1  || 1  |  0 |  1 | MOVE_PLACE | 100 |    0    |  1   |
| 1 | 0 | 1 |   0   |     0      |  0  || 1  |  0 |  1 | MOVE_PLACE | 101 |    0    |  1   |
| 1 | 0 | 1 |   0   |     0      |  1  || 1  |  0 |  1 | MOVE_PLACE | 101 |    0    |  1   |
| 1 | 0 | 1 |   0   |     1      |  0  || 1  |  1 |  0 | LOWER_PLACE| 101 |    0    |  1   |
| 1 | 0 | 1 |   0   |     1      |  1  || 1  |  1 |  0 | LOWER_PLACE| 101 |    0    |  1   |
| 1 | 0 | 1 |   1   |     0      |  0  || 1  |  0 |  1 | MOVE_PLACE | 101 |    0    |  1   |
| 1 | 0 | 1 |   1   |     0      |  1  || 1  |  0 |  1 | MOVE_PLACE | 101 |    0    |  1   |
| 1 | 0 | 1 |   1   |     1      |  0  || 1  |  1 |  0 | LOWER_PLACE| 101 |    0    |  1   |
| 1 | 0 | 1 |   1   |     1      |  1  || 1  |  1 |  0 | LOWER_PLACE| 101 |    0    |  1   |
| 1 | 1 | 0 |   0   |     0      |  0  || 1  |  1 |  0 | LOWER_PLACE| 110 |    0    |  1   |
| 1 | 1 | 0 |   0   |     0      |  1  || 1  |  1 |  0 | LOWER_PLACE| 110 |    0    |  1   |
| 1 | 1 | 0 |   0   |     1      |  0  || 1  |  1 |  1 | OPEN_GRIP  | 110 |    0    |  1   |
| 1 | 1 | 0 |   0   |     1      |  1  || 1  |  1 |  1 | OPEN_GRIP  | 110 |    0    |  1   |
| 1 | 1 | 0 |   1   |     0      |  0  || 1  |  1 |  0 | LOWER_PLACE| 110 |    0    |  1   |
| 1 | 1 | 0 |   1   |     0      |  1  || 1  |  1 |  0 | LOWER_PLACE| 110 |    0    |  1   |
| 1 | 1 | 0 |   1   |     1      |  0  || 1  |  1 |  1 | OPEN_GRIP  | 110 |    0    |  1   |
| 1 | 1 | 0 |   1   |     1      |  1  || 1  |  1 |  1 | OPEN_GRIP  | 110 |    0    |  1   |
| 1 | 1 | 1 |   0   |     0      |  0  || 0  |  0 |  0 | IDLE       | 111 |    0    |  1   |
| 1 | 1 | 1 |   0   |     0      |  1  || 0  |  0 |  0 | IDLE       | 111 |    0    |  1   |
| 1 | 1 | 1 |   0   |     1      |  0  || 0  |  0 |  0 | IDLE       | 111 |    0    |  1   |
| 1 | 1 | 1 |   0   |     1      |  1  || 0  |  0 |  0 | IDLE       | 111 |    0    |  1   |
| 1 | 1 | 1 |   1   |     0      |  0  || 1  |  1 |  1 | OPEN_GRIP  | 111 |    0    |  1   |
| 1 | 1 | 1 |   1   |     0      |  1  || 1  |  1 |  1 | OPEN_GRIP  | 111 |    0    |  1   |
| 1 | 1 | 1 |   1   |     1      |  0  || 0  |  0 |  0 | IDLE       | 111 |    0    |  1   |
| 1 | 1 | 1 |   1   |     1      |  1  || 0  |  0 |  0 | IDLE       | 111 |    0    |  1   |
```

> *Note:* The `pose` column above shows the 3-bit pose selector that gets sent to the motion controller **for the current state** (Moore outputs). `gripper` is 1 only in the CLOSE\_GRIP state. `busy` is 0 only in IDLE.

---

## Next-state boolean equations (D inputs)

The D inputs of the flip-flops equal the next-state bits:
`D_A = A+`, `D_B = B+`, `D_C = C+`.

From the transition table we can derive minimized Boolean expressions. Example simplified forms (sum-of-products):

```
D_A = (A & ~B) | (A & ~C) | (A & ~motion_done) | (B & C & grip_ok & ~A)
D_B = (B & ~C) | (A & B & ~motion_done) | (C & motion_done & ~B) | (B & ~A & ~grip_ok)
```

For `D_C` (the LSB of the next state) a direct, explicit sum-of-products (one term per current-state + trigger that makes next state odd) is:

```
D_C = (~A & ~B & ~C & start)
    | (~A & ~B &  C & ~motion_done)
    | (~A &  B & ~C & motion_done)
    | (~A &  B &  C & ~grip_ok)
    | ( A & ~B & ~C & motion_done)
    | ( A & ~B &  C & ~motion_done)
    | ( A &  B & ~C & motion_done)
    | ( A &  B &  C & ~motion_done)
```

> If desired I can run K-map simplification on `D_C` to produce a more compact boolean formula; above form is the direct readable SOP from the transition rules.

---

## Implementation notes (synthesizable / FPGA)

* Implement the state register as 3 synchronous D-FFs with a synchronous reset to `IDLE` (000).
* Moore outputs (`pose`, `gripper`, `busy`) are simple combinational logic off the state register.
* Use clear handshake semantics: when FSM sets `pose`, the motion controller must clear `motion_done` and then assert `motion_done` once the position is reached.
* Add error/timeouts: add an `ERROR` or `TIMEOUT` state if `grip_ok` or `motion_done` fail repeatedly.
* Check timing (setup/hold) for signals that come from external domains — use synchronization FIFOs or synchronizers if start/motion\_done/grip\_ok are asynchronous to the control clock.

---

## Testbench / verification (recommended)

* Write a Verilog/SystemVerilog testbench that:

  * Asserts `start` for one clock while in IDLE, then pulses `motion_done` at each pose to simulate motion completion.
  * Asserts `grip_ok` when needed during CLOSE\_GRIP.
  * Asserts/clears `motion_done` appropriately and checks the FSM transitions, outputs `pose`, `gripper`, `busy`.
* Add formal properties (SMT / PSL) e.g. `start ⇒ eventually busy = 1`, `in CLOSE_GRIP ⇒ gripper = 1`, `in OPEN_GRIP and motion_done ⇒ next state IDLE`.

---

## Files to include in repo

* `fsm.v` — synthesizable Verilog (Moore FSM). Use the state encodings above and the D equations for D-FFs.
* `fsm_tb.v` — testbench exercising `start`, `motion_done`, `grip_ok`.
* `fsm.dot` — Graphviz representation above.
* `README.md` — this file.

---
