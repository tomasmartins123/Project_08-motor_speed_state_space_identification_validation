//Project CRIA 08 - Real-Time State-Space Hardware-In-The-Loop Validation
//Runs physical motors alongside identified discrete state-space models 
//using exact parameters derived from MATLAB identification.

// Hardware Pin Assignments
const int PWM_LEFT_PIN  = 5;   // Left Motor PWM (PWM1)
const int DIR_LEFT_PIN  = A0;  // Left Motor Direction (DIR1)
const int PWM_RIGHT_PIN = 6;   // Right Motor PWM (PWM2)
const int DIR_RIGHT_PIN = A2;  // Right Motor Direction (DIR2)

const int ENC_LEFT_PIN  = 2;   // Left Encoder Interrupt (D2)
const int ENC_RIGHT_PIN = 3;   // Right Encoder Interrupt (D3)

// Encoder Interrupt Counters
volatile unsigned long count_left  = 0;
volatile unsigned long count_right = 0;
volatile unsigned long last_time_left  = 0;
volatile unsigned long last_time_right = 0;

// Identified Discrete State-Space Matrices (Ts = 0.02s)
const float A_L = 0.904837f;
const float B_L = 0.286261f;

const float A_R = 0.913101f;
const float B_R = 0.263171f;

// State Variables (Simulated Speed in RPM)
float x_sim_L = 0.0f;
float x_sim_R = 0.0f;

// Parameters
const unsigned long SAMPLE_TIME_MS = 20; // 50 Hz sampling rate
unsigned long start_time = 0;
unsigned long last_sample_time = 0;

// Left Encoder Interrupt Service Routine with 1 ms noise filter
void ISR_count_left() {
  unsigned long now = micros();
  if (now - last_time_left >= 1000) { // 1000 us = 1 ms filter for noise rejection
    count_left++;
    last_time_left = now;
  }
}

// Right Encoder Interrupt Service Routine with 1 ms noise filter
void ISR_count_right() {
  unsigned long now = micros();
  if (now - last_time_right >= 1000) { // 1000 us = 1 ms filter for noise rejection
    count_right++;
    last_time_right = now;
  }
}

void setup() {
  Serial.begin(115200);

  // Configure Motor Control Pins
  pinMode(PWM_LEFT_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  pinMode(PWM_RIGHT_PIN, OUTPUT);
  pinMode(DIR_RIGHT_PIN, OUTPUT);

  // Set Motor Direction for forward motion
  digitalWrite(DIR_LEFT_PIN, LOW);   // LOW = Forward for Left Motor
  digitalWrite(DIR_RIGHT_PIN, HIGH); // HIGH = Forward for Right Motor

  // Configure Encoder Pins
  pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN), ISR_count_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), ISR_count_right, RISING);

  // CSV Column Headers
  Serial.println("time_ms,rpm_L_real,rpm_L_sim,rpm_R_real,rpm_R_sim");

  start_time = millis();
  last_sample_time = start_time;
}

void loop() {
  unsigned long current_time = millis();

  if (current_time - last_sample_time >= SAMPLE_TIME_MS) {
    last_sample_time = current_time;
    unsigned long elapsed_ms = current_time - start_time;

    // Atomic read and reset pulse counters
    noInterrupts();
    unsigned long pulses_L = count_left;
    unsigned long pulses_R = count_right;
    count_left = 0;
    count_right = 0;
    interrupts();

    // 1. Measure Physical Speeds (RPM)
    float rpm_L_real = (float)pulses_L * 150.0f;
    float rpm_R_real = (float)pulses_R * 150.0f;

    // 2. Generate Square-Wave Step Input (Cycles every 8 seconds)
    int current_pwm = 0;
    unsigned long cycle_time = current_time % 8000;
    if (cycle_time >= 2000 && cycle_time < 6000) {
      current_pwm = 100;       //validation test, therefore a different value is tested from the one used in the identification
    } else {
      current_pwm = 0;
    }

    // Drive Physical Motors
    analogWrite(PWM_LEFT_PIN, current_pwm);
    analogWrite(PWM_RIGHT_PIN, current_pwm);

    // 3. Update Discrete State-Space Model: x[k+1] = A_d * x[k] + B_d * u[k]
    x_sim_L = A_L * x_sim_L + B_L * (float)current_pwm;
    x_sim_R = A_R * x_sim_R + B_R * (float)current_pwm;

    // Stream Data to Serial
    Serial.print(elapsed_ms);
    Serial.print(",");
    Serial.print(rpm_L_real);
    Serial.print(",");
    Serial.print(x_sim_L);
    Serial.print(",");
    Serial.print(rpm_R_real);
    Serial.print(",");
    Serial.println(x_sim_R);
  }
}