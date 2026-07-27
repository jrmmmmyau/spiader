#include <Arduino.h>

// Motor Driver Pins (TB6612FNG)
const int AIN1 = 12;
const int AIN2 = 13;
const int PWMA = 5;
const int BIN1 = 8;
const int BIN2 = 9;
const int PWMB = 6;
const int STBY = 10;

// Encoder Pins
const int ENC_A_A = 2;
const int ENC_A_B = 7;
const int ENC_B_A = 3;
const int ENC_B_B = 4;

// Encoder specs (JGB37-520)
const double CPR = 615;  // ticks per wheel revolution after gearbox

// Volatile counts (updated in ISRs)
volatile long A_pulseCount = 0;
volatile long B_pulseCount = 0;

// PID setpoints and state (in rad/s)
double left_setpoint = 0, right_setpoint = 0;
double left_velocity = 0, right_velocity = 0;

// PID constants — start here, tune later
const double Kp = 65.0, Ki = 50.0, Kd = 0.0;

// PID controller class
class PIDController {
public:
  PIDController(double Kp, double Ki, double Kd)
    : Kp(Kp), Ki(Ki), Kd(Kd), integral(0), prev_error(0) {}

  double compute(double input, double setpoint, double dt) {
    double error = setpoint - input;
    // if (abs(error) < 0.1) {  // Deadband: treat small errors as zero
    //   integral = 0;
    //   error = 0;
    // }
    integral += error * dt;
    // Anti-windup: clamp integral
    integral = constrain(integral, -100, 100);
    double derivative = (error - prev_error) / dt;
    double output = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;
    return output;
  }

  void reset() {
    integral = 0;
    prev_error = 0;
  }

private:
  double Kp, Ki, Kd;
  double integral, prev_error;
};

PIDController left_pid(Kp, Ki, Kd);
PIDController right_pid(Kp, Ki, Kd);

// Encoder ISRs
void A_ISR() {
  if (digitalRead(ENC_A_B) == LOW) A_pulseCount--;
  else A_pulseCount++;
}

void B_ISR() {
  if (digitalRead(ENC_B_B) == LOW) B_pulseCount++;
  else B_pulseCount--;
}

void setMotors(double left_output, double right_output) {
  int left_pwm = constrain((int)left_output, -255, 255);
  int right_pwm = constrain((int)right_output, -255, 255);

  // Left motor
  if (left_pwm > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, left_pwm);
  } else if (left_pwm < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -left_pwm);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }

  // Right motor
  if (right_pwm > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, right_pwm);
  } else if (right_pwm < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -right_pwm);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENC_A_A, INPUT_PULLUP);
  pinMode(ENC_A_B, INPUT_PULLUP);
  pinMode(ENC_B_A, INPUT_PULLUP);
  pinMode(ENC_B_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_A), A_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B_A), B_ISR, RISING);
}

void loop() {
  // --- Timing ---
  static unsigned long last_time = 0;
  unsigned long now = micros();
  double dt = (now - last_time) / 1000000.0;

  // Run PID at ~50 Hz (every 20 ms)
  if (dt >= 0.02) {
    last_time = now;

    // --- Compute velocities from encoder deltas ---
    static long last_A = 0, last_B = 0;

    noInterrupts();  // atomic read of volatile counters
    long cur_A = A_pulseCount;
    long cur_B = B_pulseCount;
    interrupts();

    long delta_A = cur_A - last_A;
    long delta_B = cur_B - last_B;
    last_A = cur_A;
    last_B = cur_B;

    left_velocity = -(delta_A / CPR) * 2.0 * PI / dt;
    right_velocity = -(delta_B / CPR) * 2.0 * PI / dt;

    // --- Compute PID outputs ---
    double left_output = left_pid.compute(left_velocity, left_setpoint, dt);
    double right_output = right_pid.compute(right_velocity, right_setpoint, dt);

    // --- Apply to motors ---
    setMotors(left_output, right_output);
    Serial.print("vel: ");
    Serial.print(left_velocity);
    Serial.print(" ");
    Serial.print(right_velocity);
    Serial.print(" out: ");
    Serial.print(left_output);
    Serial.print(" ");
    Serial.println(right_output);
  }

  // --- Handle serial commands ---
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'v') {
      float lv = Serial.parseFloat();
      float rv = Serial.parseFloat();
      if (lv == 0 && rv == 0) {
        setMotors(0, 0);  // Slam brakes on zero
      } else {
        left_setpoint = lv;
        right_setpoint = rv;
      }
      // Flush remainder
      while (Serial.available() > 0) Serial.read();
    } else if (cmd == 'e') {
      noInterrupts();
      long a = -A_pulseCount;
      long b = -B_pulseCount;
      interrupts();
      Serial.print(a);
      Serial.print(" ");
      Serial.println(b);
    }
  }
}