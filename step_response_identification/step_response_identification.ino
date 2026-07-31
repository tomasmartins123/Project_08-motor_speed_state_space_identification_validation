// Project CRIA 08 - Step Response Data Collection
// Applies a step PWM input to both DC motors and streams high-frequency
// timestamped speed data (RPM) via Serial in CSV format for system identification in MATLAB.

// -----------------------------------------------------------------------------
// Hardware Pin Assignments
// -----------------------------------------------------------------------------
const int PWM_LEFT_PIN  = 5;   // Left Motor PWM (PWM1)
const int DIR_LEFT_PIN  = A0;  // Left Motor Direction (DIR1)
const int PWM_RIGHT_PIN = 6;   // Right Motor PWM (PWM2)
const int DIR_RIGHT_PIN = A2;  // Right Motor Direction (DIR2)

const int ENC_LEFT_PIN  = 2;   // Left Encoder Interrupt (D2)
const int ENC_RIGHT_PIN = 3;   // Right Encoder Interrupt (D3);

// -----------------------------------------------------------------------------
// Encoder Interrupt Variables
// M/T method:
//  - sum_dt stores the accumulated time between encoder transitions.
//  - count_dt stores the number of measured periods.
//  - last_time stores the timestamp of the last valid transition.
// -----------------------------------------------------------------------------
volatile unsigned long sum_dt_left    = 0;
volatile unsigned int  count_dt_left  = 0;
volatile unsigned long last_time_left = 0;

volatile unsigned long sum_dt_right    = 0;
volatile unsigned int  count_dt_right  = 0;
volatile unsigned long last_time_right = 0;

// -----------------------------------------------------------------------------
// Last Valid RPM Values
// Used for Zero-Order Hold when no encoder transitions are detected.
// -----------------------------------------------------------------------------
float last_rpm_left = 0.0f;
float last_rpm_right = 0.0f;

// -----------------------------------------------------------------------------
// Experiment Parameters
// -----------------------------------------------------------------------------
const unsigned long SAMPLE_TIME_MS = 20;   // Sampling period (50 Hz, Ts = 0.02 s)
const int PWM_STEP_VALUE = 100;            // Step input amplitude [0,255]

unsigned long start_time = 0;
unsigned long last_sample_time = 0;

bool test_active = true;

// -----------------------------------------------------------------------------
// Left Encoder Interrupt Service Routine
// Triggered on every encoder edge (CHANGE).
// Measures the time between two consecutive valid transitions.
// Transitions separated by less than 1 ms are ignored because they are most
// likely caused by electrical noise or signal bouncing.
// -----------------------------------------------------------------------------
void ISR_count_left() {

  unsigned long now = micros();
  unsigned long dt = now - last_time_left;

  if (dt >= 1000) {

    sum_dt_left += dt;
    count_dt_left++;
    last_time_left = now;

  }
}

// -----------------------------------------------------------------------------
// Right Encoder Interrupt Service Routine
// Same logic as the left encoder.
// -----------------------------------------------------------------------------
void ISR_count_right() {

  unsigned long now = micros();
  unsigned long dt = now - last_time_right;

  if (dt >= 1000) {

    sum_dt_right += dt;
    count_dt_right++;
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
  digitalWrite(DIR_LEFT_PIN, LOW);
  digitalWrite(DIR_RIGHT_PIN, HIGH);

  // Configure Encoder Pins
  pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);

  // Interrupt on both rising and falling edges to double the number of
  // measured transitions and improve RPM resolution.
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN), ISR_count_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), ISR_count_right, CHANGE);

  // CSV header used directly by MATLAB
  Serial.println("time_ms,pwm,rpm_left,rpm_right");

  start_time = millis();
  last_sample_time = start_time;
}

void loop() {

  unsigned long current_time = millis();

  if (test_active && (current_time - last_sample_time >= SAMPLE_TIME_MS)) {

    last_sample_time = current_time;
    unsigned long elapsed_ms = current_time - start_time;

    // -------------------------------------------------------------------------
    // Copy interrupt variables atomically.
    // The accumulators are reset so that each sampling instant processes only
    // the encoder information acquired during the current 20 ms window.
    // -------------------------------------------------------------------------
    noInterrupts();

    unsigned long sum_L   = sum_dt_left;
    unsigned int  count_L = count_dt_left;
    sum_dt_left   = 0;
    count_dt_left = 0;
    unsigned long time_since_L = micros() - last_time_left;

    unsigned long sum_R   = sum_dt_right;
    unsigned int  count_R = count_dt_right;
    sum_dt_right   = 0;
    count_dt_right = 0;
    unsigned long time_since_R = micros() - last_time_right;

    interrupts();

    // -------------------------------------------------------------------------
    // 1. Measure Physical Speeds (RPM)
    // -------------------------------------------------------------------------

    // -------------------------------------------------------------------------
    // LEFT MOTOR
    //
    // Average encoder period:
    //
    //      avg_dt = Σ(dt)/N
    //
    // Instantaneous speed:
    //
    //      RPM = 1 500 000 / avg_dt
    //
    // where:
    //   60×10^6 converts microseconds into minutes.
    //   40 encoder transitions correspond to one wheel revolution.
    //   Using CHANGE interrupts doubles the number of detected transitions,
    //   giving a constant of 1 500 000.
    // -------------------------------------------------------------------------
    float rpm_L_avg = 0.0f;

    if (count_L > 0) {

      float avg_dt_L = (float)sum_L / (float)count_L;

      rpm_L_avg = 1500000.0f / avg_dt_L;

    }
    else if (time_since_L > 200000) {

      // No transitions for 200 ms -> motor assumed stopped
      rpm_L_avg = 0.0f;

    }
    else {

      // Zero-Order Hold
      rpm_L_avg = last_rpm_left;

    }

    float rpm_L_real = rpm_L_avg;
    last_rpm_left = rpm_L_real;

    // -------------------------------------------------------------------------
    // RIGHT MOTOR
    // Same processing chain as the left motor.
    // -------------------------------------------------------------------------
    float rpm_R_avg = 0.0f;

    if (count_R > 0) {

      float avg_dt_R = (float)sum_R / (float)count_R;

      rpm_R_avg = 1500000.0f / avg_dt_R;

    }
    else if (time_since_R > 200000) {

      // No transitions for 200 ms -> motor assumed stopped
      rpm_R_avg = 0.0f;

    }
    else {

      // Zero-Order Hold
      rpm_R_avg = last_rpm_right;

    }

    float rpm_R_real=rpm_R_avg;
    last_rpm_right = rpm_R_real;

  // -------------------------------------------------------------------------
  // Generate the identification step input.
  //
  // 0 - 2 s  : Motor stopped
  // 2 - 6 s  : Constant PWM step
  // After 6 s: Motor stopped and experiment finished
  // -------------------------------------------------------------------------
    int current_pwm = 0;

    if (elapsed_ms >= 2000 && elapsed_ms < 6000) {

      current_pwm = PWM_STEP_VALUE;

    }
    else if (elapsed_ms >= 6000) {

      current_pwm = 0;

      // Stop the experiment after the complete step response has been recorded
      test_active = false;

    }

    // Apply PWM to the physical motors
    analogWrite(PWM_LEFT_PIN, current_pwm);
    analogWrite(PWM_RIGHT_PIN, current_pwm);

    // -------------------------------------------------------------------------
    // Stream experimental and simulated data to the PC
    // -------------------------------------------------------------------------
    Serial.print(elapsed_ms);
    Serial.print(",");
    Serial.print(current_pwm);
    Serial.print(",");
    Serial.print(rpm_L_real, 2);
    Serial.print(",");
    Serial.println(rpm_R_real, 2);
  }

  // -------------------------------------------------------------------------
  // After the experiment finishes, keep both motors disabled.
  // -------------------------------------------------------------------------
  if (!test_active) {

    analogWrite(PWM_LEFT_PIN, 0);
    analogWrite(PWM_RIGHT_PIN, 0);

  }
}