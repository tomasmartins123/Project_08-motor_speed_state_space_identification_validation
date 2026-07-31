# Project 08: Real-Time Discrete State-Space Identification and HIL Validation

This repository contains the physical dynamic modeling, continuous and discrete state-space formulation, M/T speed measurement implementation, and real-time Hardware-in-the-Loop (HIL) cross-validation of DC motor speed dynamics on a differential wheeled robot.

---

## Project Overview

While previous iterations (Project 07) focused on model-free PID control without knowledge of motor physics, **Project 08** identifies the physical motor dynamics ($K$ and $\tau$) to build a mathematical state-space model.

The primary objective is to derive a first-order continuous dynamic model from physical principles, convert it into a discrete state-space representation at a sampling period of $T_s = 20\ \mathrm{ms}$, identify baseline system parameters under a step input of $\mathrm{PWM}=100$, and cross-validate model fidelity in real time inside an Arduino UNO under an unseen profile ($\mathrm{PWM}=150$), as well as across the full operating range (PWM 30 to 255).

---

## Physical Modeling and Derivation

### 1. Mechanical Equation of Motion (Newton's Second Law for Rotation)

Applying Newton's Second Law for rotational systems to the motor shaft:

$$
\sum T = I\alpha(t)
$$

$$
T(t)-T_{\mathrm{friction}}(t)=I\frac{d\omega(t)}{dt}
$$

Assuming linear viscous friction

$$
T_{\mathrm{friction}}(t)=b\,\omega(t)
$$

and electromagnetic torque proportional to the input

$$
T(t)=K_m\,u(t)
$$

gives

$$
K_m u(t)=I\frac{d\omega(t)}{dt}+b\omega(t)
$$

Taking the Laplace transform under zero initial conditions,

$$
K_mU(s)=IsW(s)+bW(s)
$$

which yields

$$
G(s)=\frac{W(s)}{U(s)}
=\frac{K_m}{Is+b}
$$

Dividing numerator and denominator by the viscous damping coefficient $b$ produces the standard first-order transfer function

$$
G(s)=\frac{K}{\tau s+1}
$$

where

* **Static Gain**

$$
K=\frac{K_m}{b}
$$

represents the steady-state output per unit input (RPM/PWM).

* **Time Constant**

$$
\tau=\frac{I}{b}
$$

represents the mechanical inertia of the system. After one time constant the motor reaches 63.2% of its steady-state speed, while after approximately $4\tau$ it reaches about 98%.

> **Model Order Reduction:** The real electromechanical motor is technically a second-order system. However, the electrical dynamics (armature resistance and inductance) are significantly faster than the mechanical dynamics. Their influence is therefore neglected, allowing the system to be accurately approximated by a first-order model.

---

## 2. Continuous State-Space Representation

Starting from

$$
\tau\dot{\omega}(t)+\omega(t)=Ku(t)
$$

the continuous state-space representation becomes

$$
\dot{x}(t)=Ax(t)+Bu(t)
$$

$$
y(t)=Cx(t)+Du(t)
$$

with

$$
A=-\frac1{\tau}
$$

$$
B=\frac{K}{\tau}
$$

$$
C=1
$$

$$
D=0
$$

where

* **A** describes the natural decay of the system.
* **B** determines how the input affects the motor state.
* **C** maps the state to the measured wheel speed.
* **D** is zero because there is no direct feedthrough from the PWM command to the measured output.

---

## 3. Discrete-Time State-Space Representation

Although motor dynamics are continuous, the Arduino executes the control algorithm at discrete instants every

$$
T_s=20\ \mathrm{ms}
$$

Using Zero-Order Hold discretization,

$$
x[k+1]=A_dx[k]+B_du[k]
$$

$$
y[k]=C_dx[k]+D_du[k]
$$

where

$$
A_d=e^{-T_s/\tau}
$$

$$
B_d=K\left(1-e^{-T_s/\tau}\right)
=K(1-A_d)
$$

$$
C_d=1,\qquad D_d=0
$$

---

## Speed Measurement Method: M/T (Time-Interval Principle)

Traditional pulse-counting methods (M-method) estimate speed by counting encoder pulses over a fixed sampling interval. With low-resolution encoders (20 slots per revolution), counting only one or two pulses inside a 20 ms window introduces large quantization errors of approximately 150 RPM.

To improve both resolution and identification fidelity, speed estimation was upgraded to the **M/T (Time-Interval) Method**, which measures the elapsed time between consecutive encoder transitions.

### Step 1: Consecutive Transition Time Measurement

Encoder interrupts are configured in `CHANGE` mode so that both rising and falling edges are detected.

The elapsed time between two consecutive valid transitions is

$$
dt=t_{\mathrm{now}}-t_{\mathrm{last}}
$$

using the Arduino `micros()` timer.

---

### Step 2: Noise Filtering and Transition Accumulation

To reject false transitions caused by electrical noise, intervals shorter than **1 ms** are discarded.

```cpp
if (dt >= 1000) {      // 1 ms software noise filter
    sum_dt += dt;      // Accumulate valid intervals
    count_dt++;        // Count valid transitions
    last_time = now;   // Update timestamp
}
```

The average interval between valid transitions is then computed as

$$
\Delta t_{\mathrm{avg}}
=
\frac{\sum dt}{N}
$$

where

* $\sum dt$ is the accumulated transition time.
* $N$ is the number of valid transitions detected during the sampling window.

---

### Step 3: RPM Computation

Using `CHANGE` interrupts, every encoder slot generates two transitions.

Since the encoder has **20 slots**, one wheel revolution corresponds to

$$
40
=
2\times20
$$

detected transitions.

Therefore,

$$
\mathrm{RPM}
=
\frac{60\times10^6}
{40\,\Delta t_{\mathrm{avg}}}
=
\frac{1\,500\,000}
{\Delta t_{\mathrm{avg}}}
$$

Direct measurement of transition intervals produces smooth floating-point speed estimates, eliminating the coarse quantization associated with pulse-counting methods and substantially improving identification accuracy.

---

## System Identification ($\mathrm{PWM}=100$)

System identification was performed experimentally by recording the open-loop step response using a constant PWM input of 100.

The parameters reported below correspond to the arithmetic mean obtained from five independent identification experiments.

![System Identification Step Response](step_response_identification/identification_plot.png)

### 1.1. Left Wheel Motor Model

$$\begin{aligned} A_L &= 0.857 \\ B_L &= 0.197 \end{aligned}$$

* **Steady-State Speed ($y_{ss}$):** $137.882\text{ RPM}$
* **Static Gain ($K_L$):** $1.379\text{ RPM/PWM}$
* **Time Constant ($\tau_L$):** $0.140\text{ s}$
* **Discrete Recurrence Equation:** $x_L[k+1] = 0.857 \cdot x_L[k] + 0.197 \cdot u[k]$

### 1.2. Right Wheel Motor Model

$$\begin{aligned} A_R &= 0.893 \\ B_R &= 0.151 \end{aligned}$$

* **Steady-State Speed ($y_{ss}$):** $140.098\text{ RPM}$
* **Static Gain ($K_R$):** $1.401\text{ RPM/PWM}$
* **Time Constant ($\tau_R$):** $0.192\text{ s}$
* **Discrete Recurrence Equation:** $x_R[k+1] = 0.893 \cdot x_R[k] + 0.151 \cdot u[k]$
---

## Hardware-in-the-Loop (HIL) Cross-Validation ($\text{PWM} = 150$)

Cross-validation testing was executed under an unseen validation signal ($\text{PWM} = 150$) to confirm model generalization and guard against overfitting.

![State Space HIL Validation](step_response_validation/validation_plot.png)

### Cross-Validation Behavior at $\text{PWM} = 150$

* **Linear Scaling ($K$):** When stepping up actuation by $1.5\times$ (from 100 to 150 PWM), the physical motor speeds scaled proportionally from $\approx 135\text{ RPM}$ to $\approx 207-212\text{ RPM}$, aligning with the linear discrete model prediction.
* **Transient Accuracy ($\tau$):** The exponential rise of the discrete recurrence model closely matches the experimental response during the first $0.5\text{ s}$, confirming correct continuous-to-discrete dynamic transformation.

---

## Model Validation & Operating Region Analysis

The identified discrete state-space models were validated using Hardware-in-the-Loop (HIL) experiments. For each PWM command, the response predicted by the model was compared with the speed measured by the physical DC motors through the wheel encoders.

Validation was performed across the entire motor operating range for PWM values from **30 to 255**, with increments of **20**.

### Results Across Operating Regions

The experimental results reveal three distinct operating regions:

* **Low PWM ($\approx 30 \text{ to } 50$):**
  * The motors operate close to the mechanical dead zone.
  * Encoder measurements present large variability due to low pulse frequency.
  * Static (Coulomb) friction and stiction dominate the physical response.
  * The first-order linear model does not accurately represent motor dynamics in this non-linear dead-zone region.

* **Medium PWM ($\approx 70 \text{ to } 170$):**
  * The measured response closely follows the predicted exponential behavior.
  * Both transient rise time and steady-state speeds are accurately represented by the identified models.
  * **This is the operating region where the linear approximation is most accurate.**

* **High PWM ($\approx 190 \text{ to } 255$):**
  * The model still captures the general transient response curve.
  * However, measured speed exhibits increasing dispersion around the predicted steady state.
  * This behavior is mainly caused by limited encoder resolution, electrical switching noise, supply battery voltage drop under high load, and mechanical chassis vibrations at high rotational speeds.

### Conclusions

The identified first-order discrete state-space models provide a solid approximation of DC motor dynamics over most of the usable operating range.

Although the model is less accurate near the motor dead zone and shows increasing measurement dispersion at high speeds, it correctly predicts overall dynamic behavior, including transient response and steady-state target speeds.

For control design purposes (such as state-feedback and state observers), the model can be considered valid over the **medium operating region (approximately PWM 70 to 170)**,  although the model remains representative over a much wider operating range. Between 70-170 PWM agreement between the mathematical model and physical system is strongest. Outside this interval, the model remains useful for describing general dynamics, though lower absolute precision should be expected due to hardware non-linearities and measurement limitations.

---

## Key Highlights

* **Physics-Based Identification:** Complete mechanical modeling via Newton's 2nd Law, justifying 2nd-to-1st order dynamic reduction.
* **M/T Method Integration:** Replaced fixed-window pulse counting with high-precision inter-pulse time measurement on `CHANGE` interrupt edges.
* **Identification vs Validation Separation:** System parameters identified at $\text{PWM} = 100$ and cross-validated under an unseen profile ($\text{PWM} = 150$).
* **Real-Time HIL Execution:** Discrete state-space simulation running synchronously at $50\text{ Hz}$ ($T_s = 20\text{ ms}$) on standard 8-bit microcontroller hardware.
* **Operating Range Mapping:** Comprehensive categorization of low (dead-zone), medium (linear), and high (dispersed) operating regions from PWM 30 to 255.