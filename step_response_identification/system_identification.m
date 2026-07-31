%% PROJECT CRIA 08 - System Identification & State-Space Conversion
% Reads step response CSV data, estimates first-order model parameters (K, tau),
% converts to discrete state-space representation, and validates the model.

clear; clc; close all;

%% 1. Load Experimental CSV Data
filename = 'step_response_identification_data.csv';

if ~exist(filename, 'file')
    error('File %s not found in current directory!', filename);
end

data = readtable(filename);

% Extract raw columns
t_raw = data.time_ms / 1000.0; % Convert milliseconds to seconds
u_raw = data.pwm;              % Applied PWM input [0..255]
y_L_raw = data.rpm_left;       % Left motor speed [RPM]
y_R_raw = data.rpm_right;      % Right motor speed [RPM]

Ts = 0.02; % Sampling period: 20 ms (50 Hz)

%% 2. Isolate Step Response Window (from t = 2.0s to t = 6.0s)
idx = find(t_raw >= 2.0 & t_raw <= 6.0);

t = t_raw(idx) - t_raw(idx(1)); % Normalize time vector to start at t = 0s
u = u_raw(idx);
y_L = y_L_raw(idx);
y_R = y_R_raw(idx);

u_step = max(u); % Step input magnitude (180 PWM)

%% 3. System Identification - Left Motor
% Steady-State Gain (K): Mean of last 20% of step response divided by input magnitude
N_samples = length(y_L);
y_ss_L = mean(y_L(end - round(0.2*N_samples) : end));
K_L = y_ss_L / u_step;

% Time Constant (tau): Time elapsed to reach 63.2% of steady-state speed
target_63_L = 0.632 * y_ss_L;
idx_tau_L = find(y_L >= target_63_L, 1, 'first');
tau_L = t(idx_tau_L);

%% 4. System Identification - Right Motor
y_ss_R = mean(y_R(end - round(0.2*N_samples) : end));
K_R = y_ss_R / u_step;

target_63_R = 0.632 * y_ss_R;
idx_tau_R = find(y_R >= target_63_R, 1, 'first');
tau_R = t(idx_tau_R);

%% 5. Continuous to Discrete State-Space (Left Motor)
% Continuous Transfer Function: G(s) = K / (tau*s + 1)
sys_cont_L = ss(-1/tau_L, K_L/tau_L, 1, 0);

% Discretization using Zero-Order Hold (ZOH) for Ts = 0.02s
sys_disc_L = c2d(sys_cont_L, Ts, 'zoh');

% Extract Discrete State-Space Matrices
[A_L, B_L, C_L, D_L] = ssdata(sys_disc_L);

%% 6. Continuous to Discrete State-Space (Right Motor)
sys_cont_R = ss(-1/tau_R, K_R/tau_R, 1, 0);
sys_disc_R = c2d(sys_cont_R, Ts, 'zoh');
[A_R, B_R, C_R, D_R] = ssdata(sys_disc_R);

%% 7. Console Summary Output
fprintf('\n======================================================\n');
fprintf('     IDENTIFIED PARAMETERS & DISCRETE STATE-SPACE     \n');
fprintf('======================================================\n');
fprintf(' LEFT MOTOR:\n');
fprintf('   * Steady-State Speed (y_ss): %.2f RPM\n', y_ss_L);
fprintf('   * Static Gain (K_L):          %.4f RPM/PWM\n', K_L);
fprintf('   * Time Constant (tau_L):      %.4f seconds\n', tau_L);
fprintf('   * Discrete A_d:               %.6f\n', A_L);
fprintf('   * Discrete B_d:               %.6f\n', B_L);
fprintf('   * Discrete C_d:               %.6f\n', C_L);
fprintf('   * Discrete D_d:               %.6f\n', D_L);
fprintf('------------------------------------------------------\n');
fprintf(' RIGHT MOTOR:\n');
fprintf('   * Steady-State Speed (y_ss): %.2f RPM\n', y_ss_R);
fprintf('   * Static Gain (K_R):          %.4f RPM/PWM\n', K_R);
fprintf('   * Time Constant (tau_R):      %.4f seconds\n', tau_R);
fprintf('   * Discrete A_d:               %.6f\n', A_R);
fprintf('   * Discrete B_d:               %.6f\n', B_R);
fprintf('   * Discrete C_d:               %.6f\n', C_R);
fprintf('   * Discrete D_d:               %.6f\n', D_R);
fprintf('======================================================\n\n');

%% 8. Graphical Validation (Simulated vs Real Response)
y_sim_L = lsim(sys_disc_L, u, t);
y_sim_R = lsim(sys_disc_R, u, t);

figure('Name', 'CRIA 08 - System Identification Validation', 'Color', 'w');

subplot(2,1,1);
plot(t, y_L, 'b.', 'MarkerSize', 8, 'DisplayName', 'Experimental Data'); hold on;
plot(t, y_sim_L, 'r-', 'LineWidth', 2, 'DisplayName', 'Identified Discrete Model');
grid on; title('Left Motor: Experimental Speed vs First-Order Discrete Model');
ylabel('Speed [RPM]'); legend('Location', 'southeast');

subplot(2,1,2);
plot(t, y_R, 'g.', 'MarkerSize', 8, 'DisplayName', 'Experimental Data'); hold on;
plot(t, y_sim_R, 'r-', 'LineWidth', 2, 'DisplayName', 'Identified Discrete Model');
grid on; title('Right Motor: Experimental Speed vs First-Order Discrete Model');
xlabel('Time [s]'); ylabel('Speed [RPM]'); legend('Location', 'southeast');

%% 9. Save Figure as High-Resolution Image

OUTPUT_IMAGE = 'identification_plot.png';

set(gcf,'Position',[100 100 1200 800]);   % Larger figure
exportgraphics(gcf, OUTPUT_IMAGE, 'Resolution', 300);

fprintf('Identification plot saved successfully as "%s".\n', OUTPUT_IMAGE);