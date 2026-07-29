# Project 08: Real-Time Discrete State-Space Identification and HIL Validation

This repository contains the dynamic identification, discrete state-space formulation, and real-time Hardware-in-the-Loop (HIL) cross-validation of DC motor speed dynamics on a differential wheeled robot.

---

## Project Overview

The primary objective is to derive a first-order continuous dynamic model for each motor drive, discretize the system matrices at a sampling period of $T_s = 20\text{ ms}$, and evaluate model fidelity in real time inside the Arduino UNO while executing an unseen PWM step input profile.

---

## Dynamic Modeling and Discretization

### Continuous-Time Transfer Function Model

The open-loop speed dynamics of the DC motor are represented as a first-order continuous system:

$$G(s) = \frac{Y(s)}{U(s)} = \frac{K}{\tau s + 1}$$

Where:
* $Y(s)$ is the motor rotational speed output ($\text{RPM}$).
* $U(s)$ is the continuous equivalent PWM input signal ($0\text{--}255$).
* $K$ is the steady-state static gain ($\text{RPM} / \text{PWM}$).
* $\tau$ is the system dynamic time constant ($\text{seconds}$).

---

### Discrete-Time State-Space Representation ($T_s = 20\text{ ms}$)

To execute the model synchronously within the digital control loop, the continuous system is converted into a discrete single-state space model ($x[k]$):

$$x[k+1] = A_d \cdot x[k] + B_d \cdot u[k]$$

$$y[k] = C_d \cdot x[k]$$

Where the discrete system matrices are analytically derived using zero-order hold (ZOH) discretization:

$$A_d = e^{-\frac{T_s}{\tau}}$$

$$B_d = K \left(1 - e^{-\frac{T_s}{\tau}}\right) = K (1 - A_d)$$

$$C_d = 1$$

---

## 1. Identified Discrete Parameters

System identification was performed using MATLAB Toolbox over experimental step-response data recorded at a baseline operating point ($\text{PWM} = 180$).

### 1.1. Left Wheel Motor Model

$$\begin{aligned} A_L &= 0.904837 \\ B_L &= 0.286261 \end{aligned}$$

* **Steady-State Gain ($K_L$):** $3.008\text{ RPM/PWM}$
* **Discrete Recurrence Equation:** $x_L[k+1] = 0.904837 \cdot x_L[k] + 0.286261 \cdot u[k]$

### 1.2. Right Wheel Motor Model

$$\begin{aligned} A_R &= 0.913101 \\ B_R &= 0.263171 \end{aligned}$$

* **Steady-State Gain ($K_R$):** $3.028\text{ RPM/PWM}$
* **Discrete Recurrence Equation:** $x_R[k+1] = 0.913101 \cdot x_R[k] + 0.263171 \cdot u[k]$

---

## 2. Hardware-in-the-Loop (HIL) Experimental Results

Cross-validation testing was executed under an unseen validation signal ($\text{PWM} = 100$) to verify model generalization outside identification conditions.

![State Space HIL Validation](step_response_validation/validation_plot.png)

### Graph Analysis & Physical Behavior:

* **Encoder Quantization Noise ($150\text{ RPM/pulse}$):** Because pulse counting occurs over a short sampling window ($T_s = 20\text{ ms}$), raw speed measurements are quantized into integer steps of $150\text{ RPM}$. At a physical speed of $\approx 285\text{ RPM}$, raw encoder readings alternate between $1\text{ pulse}$ ($150\text{ RPM}$) and $2\text{ pulses}$ ($300\text{ RPM}$). The physical motor velocity corresponds to the time-weighted average of these discrete states.

* **Friction Non-Linearity at Reduced Load:** At low actuation levels ($\text{PWM} = 100$), gearhead Coulomb friction exerts a higher proportional opposing torque than at the baseline operating point ($\text{PWM} = 180$). Consequently, actual physical steady-state speed ($\approx 285\text{ RPM}$) is slightly lower than the theoretical linear model prediction ($\approx 303\text{ RPM}$), demonstrating expected physical non-linear boundary behavior.

* **Real-Time Synchronism:** The real-time recurrence state equation executed inside the microcontroller tracks physical transient dynamics accurately without computational delay or buffer overflow.

---

## Key Highlights

* **Real-Time HIL Execution:** Discrete state-space simulation running synchronously at $50\text{ Hz}$ ($T_s = 20\text{ ms}$) on standard 8-bit microcontroller hardware.
* **Model Accuracy:** Low steady-state estimation error ($\approx 5\%$) during cross-validation under unseen actuation conditions.
* **Quantization Insight:** Clear experimental verification of encoder pulse quantization limits vs discrete model continuity.
* **Non-Linear Identification:** Successfully isolated linear system dynamics from low-speed gearhead friction non-linearities.