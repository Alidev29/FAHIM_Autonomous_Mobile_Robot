// =====================================================
// FAHIM robot — Arduino Nano low-level motor controller
// =====================================================
// Hardware:
//   - 2x JGB37-520 gearmotors, 12V, 333 RPM, 30:1, 11 PPR hall encoder
//     x2 quadrature decoding -> CPR = 11 * 30 * 2 = 660 per wheel rev
//   - 2x BTS7960 H-bridge drivers (RPWM + LPWM inputs)
//
// Protocol (Jetson --> Arduino, line-based, 115200 8N1):
//   L<rpm>\n    set left  wheel target RPM (signed float, e.g.  L150\n  L-80\n)
//   R<rpm>\n    set right wheel target RPM (signed float)
//   S\n         emergency stop (both setpoints -> 0)
//
// Telemetry (Arduino --> Jetson):
//   ENC,<leftTicks>,<rightTicks>,<leftRPM>,<rightRPM>\n     @ 50 Hz  (odometry)
//   DBG,<human-readable status>                             @ 2  Hz  (debug)
//
// Safety:
//   If no L/R/S received for 500 ms, setpoints are forced to 0 (watchdog).
// =====================================================

// ---- BTS7960 pin map ----
const int L_RPWM = 10;
const int L_LPWM = 9;
const int R_RPWM = 6;
const int R_LPWM = 5;

// ---- Encoder pin map ----
const int L_A = 2;   // must be interrupt-capable on Nano
const int L_B = 4;
const int R_A = 3;   // must be interrupt-capable on Nano
const int R_B = 7;

// ---- Motor / encoder constants ----
const long  ENCODER_CPR = 660;
const float MAX_RPM     = 333.0;

// ---- Loop timing (ms) ----
const unsigned long PID_INTERVAL_MS  = 50;   // was 100 — faster feedback halves windup per cycle
const unsigned long ODOM_INTERVAL_MS = 20;
const unsigned long DBG_INTERVAL_MS  = 500;
const unsigned long WATCHDOG_MS      = 500;

// ---- Stale-pulse timeout (us). If no pulse in this window, RPM = 0. ----
const unsigned long STALE_TIMEOUT_US = 200000;   // 200 ms

// ---- ISR-owned state ----
volatile unsigned long lastPulseTimeL = 0;
volatile unsigned long lastPulseTimeR = 0;
volatile unsigned long pulsePeriodL   = 0;
volatile unsigned long pulsePeriodR   = 0;
volatile long leftCount  = 0;
volatile long rightCount = 0;
volatile int8_t dirL = 0;   // +1 = forward,  -1 = backward (updated in ISR)
volatile int8_t dirR = 0;

// ---- PID state ----
struct PIDState {
  float setpoint  = 0;
  float integral  = 0;
  float prevError = 0;
  float actualRPM = 0;

  // Kp raised: 0.8*56=45 PWM was at motor deadband → stall → windup lurch.
  // 1.5*56=84 PWM starts the motor immediately on the first PID cycle.
  // Ki lowered: 0.2 accumulates 4× slower, preventing overshoot after stall.
  // Kd lowered: less noise amplification at the faster 50 ms interval.
  float Kp = 1.5;
  float Ki = 0.2;
  float Kd = 0.02;

  float buf[8] = {0,0,0,0,0,0,0,0};
  int   bufIdx = 0;

  void pushRPM(float r) {
    buf[bufIdx] = r;
    bufIdx = (bufIdx + 1) % 8;
    float s = 0;
    for (int i = 0; i < 8; i++) s += buf[i];
    actualRPM = s / 8.0;
  }

  void reset() {
    for (int i = 0; i < 8; i++) buf[i] = 0;
    bufIdx = 0;
    integral = 0;
    prevError = 0;
    actualRPM = 0;
  }
};

PIDState pidL;
PIDState pidR;

// ---- Scheduler timestamps ----
unsigned long lastPidTime  = 0;
unsigned long lastOdomTime = 0;
unsigned long lastDbgTime  = 0;
unsigned long lastCmdTime  = 0;

// ---- Serial RX line buffer ----
char rxBuf[32];
uint8_t rxLen = 0;

// ---- Forward declarations ----
float getRPM(bool isLeft);
int   computePID(PIDState &pid);
void  setMotor(int rpwmPin, int lpwmPin, int pwm);
void  ISR_Left();
void  ISR_Right();
void  readSerial();
void  processCmd(const char *line);

// =========================================
// SETUP
// =========================================
void setup() {
  Serial.begin(115200);

  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  setMotor(L_RPWM, L_LPWM, 0);
  setMotor(R_RPWM, R_LPWM, 0);

  pinMode(L_A, INPUT_PULLUP); pinMode(L_B, INPUT_PULLUP);
  pinMode(R_A, INPUT_PULLUP); pinMode(R_B, INPUT_PULLUP);

  // x2 quadrature: CHANGE on channel A only, read B for direction
  attachInterrupt(digitalPinToInterrupt(L_A), ISR_Left,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_A), ISR_Right, CHANGE);

  lastCmdTime = millis();
  Serial.println("DBG,FAHIM motor controller ready. Protocol: L<rpm>\\n R<rpm>\\n S\\n");
}

// =========================================
// MAIN LOOP
// =========================================
void loop() {
  readSerial();

  unsigned long now = millis();

  // ---- Watchdog: zero setpoints if Jetson is silent ----
  if (now - lastCmdTime > WATCHDOG_MS) {
    pidL.setpoint = 0;
    pidR.setpoint = 0;
  }

  // ---- PID update ----
  if (now - lastPidTime >= PID_INTERVAL_MS) {
    lastPidTime = now;

    float rpmL = getRPM(true);
    float rpmR = getRPM(false);
    pidL.pushRPM(rpmL);
    pidR.pushRPM(rpmR);

    int pwmL =  computePID(pidL);
    int pwmR = -computePID(pidR);  // right motor mounted mirrored -> negate

    setMotor(L_RPWM, L_LPWM, pwmL);
    setMotor(R_RPWM, R_LPWM, pwmR);
  }

  // ---- Odometry telemetry @ 50 Hz ----
  if (now - lastOdomTime >= ODOM_INTERVAL_MS) {
    lastOdomTime = now;

    long lc, rc;
    noInterrupts();
    lc = leftCount;
    rc = rightCount;
    interrupts();

    Serial.print("ENC,");
    Serial.print(lc);
    Serial.print(",");
    Serial.print(rc);
    Serial.print(",");
    Serial.print(pidL.actualRPM, 1);
    Serial.print(",");
    Serial.println(pidR.actualRPM, 1);
  }

  // ---- Debug telemetry @ 2 Hz ----
  if (now - lastDbgTime >= DBG_INTERVAL_MS) {
    lastDbgTime = now;
    Serial.print("DBG,L_sp=");  Serial.print(pidL.setpoint, 1);
    Serial.print(",L_rpm=");    Serial.print(pidL.actualRPM, 1);
    Serial.print(",R_sp=");     Serial.print(pidR.setpoint, 1);
    Serial.print(",R_rpm=");    Serial.println(pidR.actualRPM, 1);
  }
}

// =========================================
// SERIAL RX — line-buffered, non-blocking
// =========================================
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      processCmd(rxBuf);
      rxLen = 0;
    } else if (c != '\r') {
      if (rxLen < sizeof(rxBuf) - 1) {
        rxBuf[rxLen++] = c;
      } else {
        rxLen = 0;   // overflow -> drop line
      }
    }
  }
}

void processCmd(const char *line) {
  if (line[0] == '\0') return;
  char c = line[0];

  if (c == 'L' || c == 'R') {
    float val = atof(line + 1);
    val = constrain(val, -MAX_RPM, MAX_RPM);
    // Only update the setpoint — do NOT touch integral or prevError.
    //
    // cmd_vel arrives at 20 Hz; the PID runs at 10 Hz.  If we zero
    // integral on each incoming command it is wiped twice per PID cycle,
    // before the integrator can accumulate any drive.  The controller is
    // then stuck in proportional-only mode:
    //
    //   PWM = Kp * error = 0.8 * (56 - 27) ≈ 23   →  ~50% under-speed
    //
    // Letting the integral carry over allows it to build up over several
    // PID cycles until the wheel reaches the setpoint.  The ±200 clamp in
    // computePID() prevents windup.  The deadband block (|error| < 2 RPM)
    // slowly drains the integral once the wheel is on-target.
    if (c == 'L') pidL.setpoint = val;
    else          pidR.setpoint = val;
    lastCmdTime = millis();
  }
  else if (c == 'S') {
    pidL.setpoint = 0;
    pidR.setpoint = 0;
    lastCmdTime = millis();
  }
  // any other first char: silently ignore
}

// =========================================
// GET RPM — period-based, signed
// =========================================
float getRPM(bool isLeft) {
  unsigned long period, lastPulse;
  int8_t direction;

  noInterrupts();
  if (isLeft) {
    period    = pulsePeriodL;
    lastPulse = lastPulseTimeL;
    direction = dirL;
  } else {
    period    = pulsePeriodR;
    lastPulse = lastPulseTimeR;
    direction = dirR;
  }
  interrupts();

  if (period == 0 || (micros() - lastPulse) > STALE_TIMEOUT_US) {
    return 0.0;
  }
  float rpm = 60000000.0 / ((float)period * (float)ENCODER_CPR);
  return (direction >= 0) ? rpm : -rpm;
}

// =========================================
// PID — returns PWM in [-255, +255]
// =========================================
int computePID(PIDState &pid) {
  float error = pid.setpoint - pid.actualRPM;

  // Deadband: ignore tiny errors, slowly drain integral
  if (fabs(error) < 2.0) {
    error = 0;
    pid.integral *= 0.95;
  }

  pid.integral += error * (PID_INTERVAL_MS / 1000.0);
  pid.integral  = constrain(pid.integral, -100.0, 100.0);  // tighter: Ki*100=20 PWM max integral contribution

  float deriv = (error - pid.prevError) / (PID_INTERVAL_MS / 1000.0);
  deriv = constrain(deriv, -500.0, 500.0);

  float out = (pid.Kp * error)
            + (pid.Ki * pid.integral)
            + (pid.Kd * deriv);

  pid.prevError = error;
  return constrain((int)out, -255, 255);
}

// =========================================
// DRIVE MOTOR — pwm in [-255, +255]
// =========================================
void setMotor(int rpwmPin, int lpwmPin, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(rpwmPin, pwm);
    analogWrite(lpwmPin, 0);
  } else if (pwm < 0) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -pwm);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

// =========================================
// ENCODER ISRs
// Direction is captured inside the ISR (no static-history hack).
// If your motor runs backward when commanded forward, swap A/B pins
// OR flip the sign here — do NOT compensate in the main loop.
// =========================================
void ISR_Left() {
  unsigned long now = micros();
  bool a = digitalRead(L_A);
  bool b = digitalRead(L_B);
  if (a == b) { leftCount++;  dirL = +1; }
  else        { leftCount--;  dirL = -1; }
  pulsePeriodL   = now - lastPulseTimeL;
  lastPulseTimeL = now;
}

void ISR_Right() {
  unsigned long now = micros();
  bool a = digitalRead(R_A);
  bool b = digitalRead(R_B);
  // Right motor encoder wired mirrored vs. left -> inverted truth table
  if (a == b) { rightCount--; dirR = -1; }
  else        { rightCount++; dirR = +1; }
  pulsePeriodR   = now - lastPulseTimeR;
  lastPulseTimeR = now;
}
