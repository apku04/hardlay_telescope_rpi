// -------------------- Pin Definitions --------------------
#define MAIN_STOP 3

#define AZM_MOTOR 0   // Azimuth motor, use 0 for array index
#define ALT_MOTOR 1   // Altitude motor, use 1 for array index

#define DIR_INVERT_AZM  true
#define DIR_INVERT_ALT  true

// Azimuth Stepper
#define STEP_PIN_AZM      27
#define DIR_PIN_AZM       25
#define ENABLE_PIN_AZM    23
#define POT_PIN_AZM       A4
#define HOME_SWITCH_AZM   28    // NC limit switch

// Altitude Stepper
#define STEP_PIN_ALT      33
#define DIR_PIN_ALT       31
#define ENABLE_PIN_ALT    29
#define POT_PIN_ALT       A1
#define HOME_SWITCH_ALT   30    // NC limit switch

// -------------------- Movement & Speed Constants --------------------
const long  TOTAL_STEPS   = 20000;
const int FAST_DELAY_US[2] = {800, 1200};  // Example: [AZM, ALT]
const int SLOW_DELAY_US[2] = {1500, 2500}; // Example: [AZM, ALT]
const int HOMING_BACKOFF_STEPS[2] = {100, 200}; // Example: [AZM, ALT]

const int   DEADBAND      = 200;

// -------------------- State & Calibration --------------------
long current_position[2] = {0, 0};         // [AZM, ALT]
long offset[2]           = {TOTAL_STEPS / 2, TOTAL_STEPS / 8};
bool didCenterMove = false;

bool isStopped = false;
bool lastButtonState = HIGH;

// -------------------- Pin Arrays for Clean Code --------------------
const int STEP_PIN[2]     = {STEP_PIN_AZM, STEP_PIN_ALT};
const int DIR_PIN[2]      = {DIR_PIN_AZM, DIR_PIN_ALT};
const int ENABLE_PIN[2]   = {ENABLE_PIN_AZM, ENABLE_PIN_ALT};
const int POT_PIN[2]      = {POT_PIN_AZM, POT_PIN_ALT};
const int HOME_SWITCH[2]  = {HOME_SWITCH_AZM, HOME_SWITCH_ALT};
const bool DIR_INVERT[2]  = {DIR_INVERT_AZM, DIR_INVERT_ALT};

// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Motor pins
  for (int i = 0; i < 2; i++) {
    pinMode(STEP_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
    pinMode(ENABLE_PIN[i], OUTPUT);
    pinMode(HOME_SWITCH[i], INPUT_PULLUP);
  }
  pinMode(MAIN_STOP, INPUT_PULLUP);

  // Disable motors
  disableMotor(AZM_MOTOR);
  disableMotor(ALT_MOTOR);

  // Homing
  homeAxis(AZM_MOTOR);
  homeAxis(ALT_MOTOR);

  // Move to center
  goToCenterOffset(AZM_MOTOR, offset[AZM_MOTOR], FAST_DELAY_US[AZM_MOTOR]);
  goToCenterOffset(ALT_MOTOR, offset[ALT_MOTOR], FAST_DELAY_US[ALT_MOTOR]);
  

  didCenterMove = true;
}

// ------------------------------------------------------------
void loop() {
  updateStopToggle();

  if (isStopped){
    disableMotor(AZM_MOTOR);
    disableMotor(ALT_MOTOR);
    return;  // If in stop state, skip rest
  } 

  int pot_azm = readSmoothedPot(POT_PIN[AZM_MOTOR]);
  int pot_alt = readSmoothedPot(POT_PIN[ALT_MOTOR]);

  long span = TOTAL_STEPS;
  long target_azm = offset[AZM_MOTOR] + map(pot_azm, 0, 1023, -span / 2, span / 2);
  long target_alt = offset[ALT_MOTOR] + map(pot_alt, 0, 1023, -span / 2, span / 2);

  moveStepperWithSpeed(AZM_MOTOR, target_azm, SLOW_DELAY_US[AZM_MOTOR]);
  moveStepperWithSpeed(ALT_MOTOR, target_alt, SLOW_DELAY_US[ALT_MOTOR]);
}

// ------------------------------------------------------------
void updateStopToggle() {
  bool buttonState = digitalRead(MAIN_STOP);

  // Detect falling edge: HIGH -> LOW (button just pressed)
  if (buttonState == LOW && lastButtonState == HIGH) {
    isStopped = !isStopped;   // Toggle state
    delay(50);                // Simple debounce
  }
  lastButtonState = buttonState;
}

// ---------------------- Stepper Functions ------------------------
void homeAxis(int axis) {
  // Use per-axis homing speeds and backoff steps if desired
  int fast_speed = FAST_DELAY_US[axis];
  int slow_speed = SLOW_DELAY_US[axis];
  int backoff_steps = HOMING_BACKOFF_STEPS[axis];

  enableMotor(axis);

  // 1. Fast approach towards the switch
  digitalWrite(DIR_PIN[axis], DIR_INVERT[axis] ? HIGH : LOW);
  while (digitalRead(HOME_SWITCH[axis]) == LOW) {
    stepMotor(axis, fast_speed);
  }
  Serial.println("Switch hit (fast)!");

  // 2. Back off from the switch
  digitalWrite(DIR_PIN[axis], DIR_INVERT[axis] ? LOW : HIGH); // Reverse direction
  for (int i = 0; i < backoff_steps; i++) {
    stepMotor(axis, fast_speed);
  }
  while (digitalRead(HOME_SWITCH[axis]) == HIGH) {
    stepMotor(axis, fast_speed);
  }
  Serial.println("Switch released.");

  // 3. Slow approach towards the switch
  digitalWrite(DIR_PIN[axis], DIR_INVERT[axis] ? HIGH : LOW);
  while (digitalRead(HOME_SWITCH[axis]) == LOW) {
    stepMotor(axis, slow_speed);
  }
  Serial.println("Switch hit (slow)!");

  // 4. Set current position
  current_position[axis] = 0;
  disableMotor(axis);

  Serial.println("Homing done!");
}



void goToCenterOffset(int axis, long target, int delay_us) {
  long* cur      = &current_position[axis];
  int  stepPin   = STEP_PIN[axis];
  int  dirPin    = DIR_PIN[axis];
  int  enablePin = ENABLE_PIN[axis];
  bool invert    = DIR_INVERT[axis];

  digitalWrite(enablePin, LOW);
  while (*cur != target) {
    if (invert) {
      digitalWrite(dirPin, (*cur < target) ? LOW : HIGH);
    } else {
      digitalWrite(dirPin, (*cur < target) ? HIGH : LOW);
    }
    *cur += (*cur < target) ? 1 : -1;
    digitalWrite(stepPin, HIGH);  delayMicroseconds(delay_us);
    digitalWrite(stepPin, LOW);   delayMicroseconds(delay_us);
  }
  digitalWrite(enablePin, HIGH);
}

void moveStepperWithSpeed(int axis, long target, int delay_us) {
  long* cur      = &current_position[axis];
  int  stepPin   = STEP_PIN[axis];
  int  dirPin    = DIR_PIN[axis];
  int  enablePin = ENABLE_PIN[axis];
  bool invert    = DIR_INVERT[axis];

  if (abs(target - *cur) < DEADBAND) {
    digitalWrite(enablePin, HIGH);
    return;
  }

  digitalWrite(enablePin, LOW);
  if (invert) {
    digitalWrite(dirPin, (*cur < target) ? LOW : HIGH);
  } else {
    digitalWrite(dirPin, (*cur < target) ? HIGH : LOW);
  }
  *cur += (*cur < target) ? 1 : -1;
  digitalWrite(stepPin, HIGH);  delayMicroseconds(delay_us);
  digitalWrite(stepPin, LOW);   delayMicroseconds(delay_us);
}

void stepMotor(int axis, int delay_us) {
  digitalWrite(STEP_PIN[axis], HIGH);  delayMicroseconds(delay_us);
  digitalWrite(STEP_PIN[axis], LOW);   delayMicroseconds(delay_us);
}

void enableMotor(int axis) {
  digitalWrite(ENABLE_PIN[axis], LOW);
}
void disableMotor(int axis) {
  digitalWrite(ENABLE_PIN[axis], HIGH);
}

// ------------------- Utility --------------------
int readSmoothedPot(int pin) {
  long sum = 0;
  const int samples = 20;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(500);
  }
  return sum / samples;
}
