#Project CRIA 08 - State-Space Model Validation Plotter
#Loads validation experiment CSV data and generates a high-resolution 
#comparison plot between physical encoder measurements and internal Arduino state-space predictions.


import matplotlib.pyplot as plt
import pandas as pd

# ==========================================
# 1. Configuration
# ==========================================
CSV_FILE = "step_response_validation_data.csv"
OUTPUT_IMAGE = "validation_plot.png"

# ==========================================
# 2. Data Loading & Processing
# ==========================================
print(f"Loading validation data from '{CSV_FILE}'...")
data = pd.read_csv(CSV_FILE)

# Convert time from milliseconds to seconds
data["time_sec"] = data["time_ms"] / 1000.0

# Filter only the active step window (from 2.0s until just BEFORE PWM drops at 5.95s)
data = data[(data["time_sec"] >= 2.0) & (data["time_sec"] < 5.95)].copy()

#  Normalization: Shift time so the step starts exactly at t = 0.0s
data["time_sec"] = data["time_sec"] - 2.0

# Convert exact CSV columns to NumPy arrays
time_sec = data["time_sec"].to_numpy()
rpm_L_real = data["rpm_L_real"].to_numpy()
rpm_L_sim = data["rpm_L_sim"].to_numpy()
rpm_R_real = data["rpm_R_real"].to_numpy()
rpm_R_sim = data["rpm_R_sim"].to_numpy()

# ==========================================
# 3. Plotting
# ==========================================
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# --- Left Motor Subplot ---
ax1.scatter(
    time_sec,
    rpm_L_real,
    color="indianred",
    alpha=0.6,
    s=20,
    label="Physical Encoder (Left)",
)
ax1.plot(
    time_sec,
    rpm_L_sim,
    color="#0072BD",
    linewidth=2.5,
    label="Discrete Model $x[k]$ (Left)",
)
ax1.set_title("Hardware-in-the-Loop Validation: Left Motor", fontweight="bold")
ax1.set_ylabel("Speed [RPM]", fontweight="bold")
ax1.set_xlim(0, 4.0)
ax1.grid(True, linestyle="--", alpha=0.5)
ax1.legend(loc="lower right")

# --- Right Motor Subplot ---
ax2.scatter(
    time_sec,
    rpm_R_real,
    color="mediumseagreen",
    alpha=0.6,
    s=20,
    label="Physical Encoder (Right)",
)
ax2.plot(
    time_sec,
    rpm_R_sim,
    color="#D95319",
    linewidth=2.5,
    label="Discrete Model $x[k]$ (Right)",
)
ax2.set_title(
    "Hardware-in-the-Loop Validation: Right Motor", fontweight="bold"
)
ax2.set_xlabel("Time [s]", fontweight="bold")
ax2.set_ylabel("Speed [RPM]", fontweight="bold")
ax2.set_xlim(0, 4.0)
ax2.grid(True, linestyle="--", alpha=0.5)
ax2.legend(loc="lower right")

# Title and Layout Adjustments
fig.suptitle(
    "Hardware-in-the-Loop Validation: Physical Motors vs Discrete State-Space Models",
    fontsize=14,
    fontweight="bold",
)
plt.tight_layout()

# Save image and render
plt.savefig(OUTPUT_IMAGE, dpi=300)
print(f"Plot saved successfully as '{OUTPUT_IMAGE}'.")
plt.show()