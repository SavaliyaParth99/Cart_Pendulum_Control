# Cart Pendulum Control

This project presents a **real-time cart–pendulum control system** implemented on a **Xilinx Zynq SoC (Arty Z7)**. The work covers **system modeling, control design, fixed-point implementation, and hardware validation** on an actual test stand.

---

## Real Hardware Test Bench Setup

![Cart Pendulum Test Bench](https://github.com/SavaliyaParth99/Cart_Pendulum_Control/blob/main/src/Testbench.jpg)

## What This Project Does

- Controls **cart position**
- Actively **stabilizes pendulum oscillations**
- Handles **real-world disturbances**
- Executes in **real time on Zynq hardware**
- Provides **live monitoring via GUI**

This system is a practical representation of an **overhead crane control problem** used in industry.

---

## Core Technologies Used

- **Xilinx Zynq-7000 (Arty Z7)**
- **MATLAB & Simulink**
- **Embedded C (Fixed-Point)**
- **VHDL (SPI & Custom IP)**
- **Java (GUI)**
- **Incremental Encoders, DAC, ADC, DC Servo Motor**

---

## Main Technical Work

### 1. System Modeling
- Nonlinear modeling using **Lagrange equations**
- Linearization and **state-space representation**
- Four-state system:
  - Cart position
  - Cart velocity
  - Pendulum angle
  - Pendulum angular velocity

---

### 2. Controller Design
- **State-feedback controller** using pole placement
- **PI position controller** for zero steady-state error
- **PI speed controller** for safe referencing phase
- Both **continuous and discrete implementations**

---

### 3. Fixed-Point Embedded Implementation
- All controllers converted from floating-point to **fixed-point**
- Designed for **real-time execution on Zynq ARM processor**
- Force output generated using **12-bit DAC**

---

### 4. Software-in-the-Loop (SIL) Verification
- Controllers written in **C**
- Integrated into Simulink using **S-functions**
- Outputs verified against original Simulink models

---

### 5. Hardware Implementation
- Deployed on **Zynq-7000 SoC**
- Custom **AXI-based IP**
- **SPI communication** for DAC and ADC
- Encoder-based position and angle feedback
- Real-time interrupt-driven control

---

### 6. Graphical User Interface (GUI)
- Built in **Java**
- Live display of:
  - Cart position & velocity
  - Pendulum angle & velocity
  - Applied force
- Functions:
  - Referencing
  - Start/Stop control
  - Data logging
  - Graph plotting


---

## 🔗 Main Real-Time Control C Code

The **core real-time control logic** (interrupts, referencing phase, and position control state machine) is implemented in this file:

👉 **Main Embedded Control File:**  
https://github.com/SavaliyaParth99/Cart_Pendulum_Control/blob/main/Cpc/C_code/cpcivpmn.c

This file contains:
- Referencing state machine
- Speed control during referencing
- Position control loop
- Encoder reading & DAC output
- Real-time interrupt handling

---

## C Code vs Simulink Comparison (Validation)

To verify real-time embedded behavior, the **fixed-point C controller outputs were directly compared with Simulink models** for both **position control** and **speed control**.

---

### 🔹 Position Control: C Code vs Simulink

This comparison validates that the **real-time cart position controller implemented in fixed-point C** matches the designed Simulink reference model.

![Position Control Simulink vs C Comparison](https://github.com/SavaliyaParth99/Cart_Pendulum_Control/blob/main/src/Cpc_simulink/Pos_Cotrol_Simulink_Model.png)

---

### 🔹 Speed Control: C Code vs Simulink

This comparison validates that the **embedded PI speed controller implemented in C** behaves identically to the Simulink model during the referencing phase.

![Speed Control Simulink vs C Comparison](https://github.com/SavaliyaParth99/Cart_Pendulum_Control/blob/main/src/Cpc_simulink/Speed_Cotrol_Simulink_Model.png)

---

## Output Graph Comparison (Simulink vs Real-Time C)

These graphs compare the **real-time fixed-point C controller output** with the **Simulink reference model**, validating dynamic behavior for both **position** and **speed**.

---

### 🔹 Position Output Comparison

![Position Output Comparison](https://github.com/SavaliyaParth99/Cart_Pendulum_Control/blob/main/src/Cpc_simulink/Pos_Comparison_Graph.png)

---

### 🔹 Speed Output Comparison

![Speed Output Comparison](https://github.com/SavaliyaParth99/Cart_Pendulum_Control/blob/main/src/Cpc_simulink/Speed_Comparison_Graph.png)

---

✅ These results confirm that the **real-time fixed-point C implementation accurately reproduces the Simulink controller behavior**, validating correctness for hardware deployment.


## Final Outcome

- ✅ Real-time pendulum stabilization achieved  
- ✅ Accurate cart positioning under disturbances  
- ✅ Validated on real hardware, not only simulation  

---


---

## Sequential Process Flow

The following flow chart illustrates the **complete real-time execution sequence** of the Cart Pendulum Control system, from system start-up to active closed-loop control:

![Cart Pendulum Control Sequential Flow](https://github.com/SavaliyaParth99/Cart_Pendulum_Control/blob/main/src/Sequential_Flow_Cpc.png)

This process includes:
- System initialization
- Referencing using limit switches
- Speed control during referencing
- Activation of position control
- Continuous real-time feedback loop execution

