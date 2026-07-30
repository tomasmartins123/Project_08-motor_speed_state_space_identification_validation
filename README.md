# Project 08: Real-Time Discrete State-Space Identification and HIL Validation

This repository contains the physical dynamic modeling, continuous and discrete state-space formulation, and real-time Hardware-in-the-Loop (HIL) cross-validation of DC motor speed dynamics on a differential wheeled robot.

---

## Project Overview

While previous iterations (Project 07) focused on model-free PID control without knowledge of motor physics, **Project 08** identifies the physical motor dynamics ($K$ and $\tau$) to build a mathematical state-space model. 

The primary objective is to derive a first-order continuous dynamic model from physical principles, convert it into a discrete state-space representation at a sampling period of $T_s = 20\text{ ms}$, and validate model fidelity in real time inside an Arduino UNO executing an unseen PWM step input profile.

---

## Physical Modeling and Derivation

### 1. Mechanical Equation of Motion (Newton's 2nd Law for Rotation)

Applying Newton's Second Law for rotational systems to the motor shaft:

$$\sum T = I \cdot \alpha(t)$$

$$T(t) - T_{\text{friction}}(t) = I \frac{d\omega(t)}{dt}$$

Assuming linear viscous friction $T_{\text{friction}}(t) = b \cdot \omega(t)$ and electromagnetic torque proportional to input $u(t)$ ($T(t) = K_m \cdot u(t)$):

$$K_m u(t) = I \frac{d\omega(t)}{dt} + b \omega(t)$$

Taking the Laplace transform under zero initial conditions:

$$K_m U(s) = I s W(s) + b W(s) \implies \frac{W(s)}{U(s)} = \frac{K_m}{I s + b}$$

Dividing numerator and denominator by the viscous damping coefficient $b$ yields the standard first-order Transfer Function:

$$G(s) = \frac{W(s)}{U(s)} = \frac{K}{\tau s + 1}$$

Where the physical parameters are defined as:
* **Static Gain ($K = \frac{K_m}{b}$):** Represents steady-state output per unit input ($\text{RPM} / \text{PWM}$) at $s = 0$.
* **Time Constant ($\tau = \frac{I}{b}$):** Represents mechanical system inertia, defined as the time required to reach $63.2\%$ of the steady-state velocity ($y_{ss}$). At $t = 4\tau$, the system reaches $\approx 98\%$ of its final speed.

> **Model Order Reduction:** The real electro-mechanical motor is technically a 2nd-order system. However, because electrical dynamics (inductance/resistance time constants) are virtually instantaneous compared to mechanical inertia, the electrical domain collapses, simplifying the system to 1st order without significant loss of accuracy.

---

### 2. Continuous State-Space Representation

Converting the differential equation $\tau \dot{\omega}(t) + \omega(t) = K u(t)$ into continuous state-space form ($\dot{x}(t) = A x(t) + B u(t)$, $y(t) = C x(t) + D u(t)$):

$$\dot{x}(t) = \left[-\frac{1}{\tau}\right] x(t) + \left[\frac{K}{\tau}\right] u(t)$$

$$y(t) = [1] x(t) + [0] u(t)$$

* **$A$ (Dynamic Matrix):** Describes natural system dissipation ($-\frac{1}{\tau}$). Its eigenvalue defines the continuous system pole.
* **$B$ (Input Matrix):** Quantifies how input commands directly alter the state ($\frac{K}{\tau}$).
* **$C$ (Output Matrix):** Maps internal state $x(t)$ to encoder speed measurements $y(t)$ ($C = 1$).
* **$D$ (Direct Transmission Matrix):** Set to zero ($D = 0$), indicating no instantaneous feedthrough from input to output.

> **Generalization across Operating Points:** Because physical parameters $K = \frac{K_m}{b}$ and $\tau = \frac{I}{b}$ are fixed physical constants of the motor setup, matrices $A$ and $B$ are input-invariant. A model identified at $\text{PWM} = 180$ remains valid for any arbitrary PWM level (e.g., $\text{PWM} = 100$).

---

### 3. Discrete-Time State-Space Representation ($T_s = 20\text{ ms}$)

While physical motor dynamics are continuous, microcontrollers execute digitally in discrete time steps ($T_s = 20\text{ ms}$). To execute synchronously within the control loop, the system is discretized via Zero-Order Hold (ZOH):

$$x[k+1] = A_d \cdot x[k] + B_d \cdot u[k]$$

$$y[k] = C_d \cdot x[k] + D_d \cdot u[k]$$

Analytically derived discrete matrices:

$$A_d = e^{-\frac{T_s}{\tau}}$$

$$B_d = K \left(1 - e^{-\frac{T_s}{\tau}}\right) = K (1 - A_d)$$

$$C_d = 1, \quad D_d = 0$$

---

## 1. Identified Discrete Parameters

System identification was performed using experimental step-response data recorded at $\text{PWM} = 180$.

### 1.1. Left Wheel Motor Model

$$\begin{aligned} A_L &= 0.904837 \\ B_L &= 0.286261 \end{aligned}$$

* **Steady-State Gain ($K_L$):** $3.008\text{ RPM/PWM}$
* **Time Constant ($\tau_L$):** $\approx 0.203\text{ s}$
* **Discrete Recurrence Equation:** $x_L[k+1] = 0.904837 \cdot x_L[k] + 0.286261 \cdot u[k]$

### 1.2. Right Wheel Motor Model

$$\begin{aligned} A_R &= 0.913101 \\ B_R &= 0.263171 \end{aligned}$$

* **Steady-State Gain ($K_R$):** $3.028\text{ RPM/PWM}$
* **Discrete Recurrence Equation:** $x_R[k+1] = 0.913101 \cdot x_R[k] + 0.263171 \cdot u[k]$

---

### 📌 Practical Estimation Note on $\tau$ and Quantization Limit
During identification, $\tau$ is estimated by searching for the first discrete sample crossing $63.2\%$ of steady-state speed (`y >= 0.632 * y_ss`). Due to encoder speed quantization, this single-sample lookup detects the first step over the threshold (e.g., $450\text{ RPM}$ crossing a $344\text{ RPM}$ target). While linear interpolation or regression over the transient rise would yield higher theoretical precision ($\approx 0.203\text{ s}$ vs $\approx 0.22\text{ s}$), the resulting variance ($\approx 17\text{ ms}$) is smaller than a single sampling period ($T_s = 20\text{ ms}$), making discrete lookup errors negligible for real-time closed-loop control.

---

## 2. Hardware-in-the-Loop (HIL) Experimental Results

Cross-validation testing was executed under an unseen validation signal ($\text{PWM} = 100$) to confirm model generalization and guard against overfitting.

![State Space HIL Validation](step_response_validation/validation_plot.png)

### Graph Analysis & Physical Behavior

* **Encoder Quantization Noise ($150\text{ RPM/pulse}$):** Because pulse counting occurs over a short sampling window ($T_s = 20\text{ ms}$), raw speed measurements are quantized into discrete steps of $150\text{ RPM}$. At a physical speed of $\approx 285\text{ RPM}$, raw encoder readings alternate between $1\text{ pulse}$ ($150\text{ RPM}$) and $2\text{ pulses}$ ($300\text{ RPM}$). Physical velocity corresponds to the time-weighted average of these discrete states.

* **Friction Non-Linearity at Reduced Load:** At lower actuation levels ($\text{PWM} = 100$), gearhead Coulomb friction exerts a higher proportional opposing torque than at baseline ($\text{PWM} = 180$). Consequently, actual physical steady-state speed ($\approx 285\text{ RPM}$) is slightly lower than linear model prediction ($\approx 303\text{ RPM}$), demonstrating expected physical boundary non-linearities.

* **Real-Time Synchronism:** The real-time recurrence state equation executed inside the microcontroller tracks physical transient dynamics accurately without computational delay or buffer overflow.

---

## Key Highlights

* **Physics-Based Identification:** Complete mechanical modeling via Newton's 2nd Law, justifying 2nd-to-1st order dynamic reduction.
* **Real-Time HIL Execution:** Discrete state-space simulation running synchronously at $50\text{ Hz}$ ($T_s = 20\text{ ms}$) on standard 8-bit microcontroller hardware.
* **Cross-Validation Rigor:** Model robustness verified under unseen operating points ($\text{PWM} = 100$) to prevent overfitting.
* **Quantization Insight:** Detailed resolution analysis comparing continuous physics against discrete encoder sampling bounds.
* **Non-Linear Identification:** Successfully isolated linear system dynamics from low-speed gearhead friction non-linearities.
