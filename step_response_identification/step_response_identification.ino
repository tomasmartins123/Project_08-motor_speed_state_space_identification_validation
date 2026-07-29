// Project CRIA 08 - Step Response Data Collection
// Applies a step PWM input to both DC motors and streams high-frequency 
// timestamped speed data (RPM) via Serial in CSV format for system identification in MATLAB.

// Hardware Pin Assignments
const int PWM_LEFT_PIN  = 5;   // Left Motor PWM (PWM1)
const int DIR_LEFT_PIN  = A0;  // Left Motor Direction (DIR1)
const int PWM_RIGHT_PIN = 6;   // Right Motor PWM (PWM2)
const int DIR_RIGHT_PIN = A2;  // Right Motor Direction (DIR2)

const int ENC_LEFT_PIN  = 2;   // Left Encoder Interrupt (D2)
const int ENC_RIGHT_PIN = 3;   // Right Encoder Interrupt (D3)

// Encoder Interrupt Counters
volatile unsigned long count_left = 0;
volatile unsigned long count_right = 0;
volatile unsigned long last_time_left = 0;
volatile unsigned long last_time_right = 0;

// Parameters
const unsigned long SAMPLE_TIME_MS = 20;  // 50 Hz sampling rate (Ts = 0.02s)
const int PWM_STEP_VALUE = 180;           // Step amplitude [0,255]

unsigned long last_sample_time = 0;
bool test_active = true;

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

  // Print CSV Header for MATLAB ingestion
  Serial.println("time_ms,pwm,rpm_left,rpm_right");
  start_time = millis();
  last_sample_time = start_time;
}

void loop() {
  unsigned long current_time = millis();

  if (test_active && (current_time - last_sample_time >= SAMPLE_TIME_MS)) {
    last_sample_time = current_time;

    // Atomic read and reset of encoder pulse counts
    noInterrupts();
    unsigned long pulses_L = count_left;
    unsigned long pulses_R = count_right;
    count_left = 0;
    count_right = 0;
    interrupts();

    // Calculate speed in RPM for a 20-slot encoder disc
    // RPM = (pulses / 20 slots) * (60000 ms / SAMPLE_TIME_MS)
    // For SAMPLE_TIME_MS = 20 ms: RPM = pulses * 150.0
    float rpm_L = (float)pulses_L * 150.0f;
    float rpm_R = (float)pulses_R * 150.0f;

    // Determine PWM value based on experiment timeline
    int current_pwm = 0;
    if (current_time >= 2000 && current_time < 6000) {
      current_pwm = PWM_STEP_VALUE; // Apply step input (from 2.0s to 6.0s)
    } else if (current_time >= 6000) {
      current_pwm = 0;
      test_active = false;          // End of test sequence
    }

    // Apply motor actuation
    analogWrite(PWM_LEFT_PIN, current_pwm);
    analogWrite(PWM_RIGHT_PIN, current_pwm);

    // Stream CSV row via Serial
    Serial.print(current_time);
    Serial.print(",");
    Serial.print(current_pwm);
    Serial.print(",");
    Serial.print(rpm_L, 2);
    Serial.print(",");
    Serial.println(rpm_R, 2);
  }

  if (!test_active) {
    analogWrite(PWM_LEFT_PIN, 0);
    analogWrite(PWM_RIGHT_PIN, 0);
  }
}