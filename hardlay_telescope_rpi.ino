// -------------------- Pin Definitions --------------------
constexpr int MAIN_STOP = 3;

enum Axis { AZM = 0, ALT, AXIS_COUNT };

struct StepperAxis {
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t enablePin;
  uint8_t potPin;
  uint8_t homeSwitch;
  bool    dirInvert;
  long    position;
  long    offset;
  int     fastDelay;
  int     slowDelay;
  int     backoffSteps;
};

constexpr long TOTAL_STEPS = 20000;

StepperAxis axes[AXIS_COUNT] = {
  {27, 25, 23, A4, 28, true, 0, TOTAL_STEPS / 2, 800, 1500, 100},
  {33, 31, 29, A1, 30, true, 0, TOTAL_STEPS / 8, 1200, 2500, 200}
};

// -------------------- Movement & Speed Constants --------------------
constexpr int DEADBAND = 200;

// -------------------- State & Calibration --------------------
bool didCenterMove   = false;
bool isStopped       = false;
bool lastButtonState = HIGH;

// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < AXIS_COUNT; ++i) {
    pinMode(axes[i].stepPin, OUTPUT);
    pinMode(axes[i].dirPin, OUTPUT);
    pinMode(axes[i].enablePin, OUTPUT);
    pinMode(axes[i].homeSwitch, INPUT_PULLUP);
    digitalWrite(axes[i].enablePin, HIGH);
  }
  pinMode(MAIN_STOP, INPUT_PULLUP);

  homeAxis(AZM);
  homeAxis(ALT);

  moveToTarget(AZM, axes[AZM].offset, axes[AZM].fastDelay);
  moveToTarget(ALT, axes[ALT].offset, axes[ALT].fastDelay);

  didCenterMove = true;
}

// ------------------------------------------------------------
void loop() {
  updateStopToggle();

  if (isStopped) {
    disableMotor(AZM);
    disableMotor(ALT);
    return;  // Skip rest if stopped
  }

  int potAzm = readSmoothedPot(axes[AZM].potPin);
  int potAlt = readSmoothedPot(axes[ALT].potPin);

  long span = TOTAL_STEPS;
  long targetAzm = axes[AZM].offset + map(potAzm, 0, 1023, -span / 2, span / 2);
  long targetAlt = axes[ALT].offset + map(potAlt, 0, 1023, -span / 2, span / 2);

  moveStepperWithSpeed(AZM, targetAzm, axes[AZM].slowDelay);
  moveStepperWithSpeed(ALT, targetAlt, axes[ALT].slowDelay);
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
void homeAxis(Axis idx) {
  StepperAxis &ax = axes[idx];

  enableMotor(idx);

  // 1. Fast approach towards the switch
  digitalWrite(ax.dirPin, ax.dirInvert ? HIGH : LOW);
  while (digitalRead(ax.homeSwitch) == LOW) {
    stepMotor(idx, ax.fastDelay);
  }
  Serial.println("Switch hit (fast)!");

  // 2. Back off from the switch
  digitalWrite(ax.dirPin, ax.dirInvert ? LOW : HIGH);
  for (int i = 0; i < ax.backoffSteps; ++i) {
    stepMotor(idx, ax.fastDelay);
  }
  while (digitalRead(ax.homeSwitch) == HIGH) {
    stepMotor(idx, ax.fastDelay);
  }
  Serial.println("Switch released.");

  // 3. Slow approach towards the switch
  digitalWrite(ax.dirPin, ax.dirInvert ? HIGH : LOW);
  while (digitalRead(ax.homeSwitch) == LOW) {
    stepMotor(idx, ax.slowDelay);
  }
  Serial.println("Switch hit (slow)!");

  // 4. Set current position
  ax.position = 0;
  disableMotor(idx);

  Serial.println("Homing done!");
}



void moveToTarget(Axis idx, long target, int delay_us) {
  StepperAxis &ax = axes[idx];
  digitalWrite(ax.enablePin, LOW);
  while (ax.position != target) {
    if (ax.dirInvert) {
      digitalWrite(ax.dirPin, (ax.position < target) ? LOW : HIGH);
    } else {
      digitalWrite(ax.dirPin, (ax.position < target) ? HIGH : LOW);
    }
    ax.position += (ax.position < target) ? 1 : -1;
    digitalWrite(ax.stepPin, HIGH);  delayMicroseconds(delay_us);
    digitalWrite(ax.stepPin, LOW);   delayMicroseconds(delay_us);
  }
  digitalWrite(ax.enablePin, HIGH);
}

void moveStepperWithSpeed(Axis idx, long target, int delay_us) {
  StepperAxis &ax = axes[idx];

  if (abs(target - ax.position) < DEADBAND) {
    digitalWrite(ax.enablePin, HIGH);
    return;
  }

  digitalWrite(ax.enablePin, LOW);
  if (ax.dirInvert) {
    digitalWrite(ax.dirPin, (ax.position < target) ? LOW : HIGH);
  } else {
    digitalWrite(ax.dirPin, (ax.position < target) ? HIGH : LOW);
  }
  ax.position += (ax.position < target) ? 1 : -1;
  digitalWrite(ax.stepPin, HIGH);  delayMicroseconds(delay_us);
  digitalWrite(ax.stepPin, LOW);   delayMicroseconds(delay_us);
}

void stepMotor(Axis idx, int delay_us) {
  StepperAxis &ax = axes[idx];
  digitalWrite(ax.stepPin, HIGH);  delayMicroseconds(delay_us);
  digitalWrite(ax.stepPin, LOW);   delayMicroseconds(delay_us);
}

void enableMotor(Axis idx) {
  digitalWrite(axes[idx].enablePin, LOW);
}
void disableMotor(Axis idx) {
  digitalWrite(axes[idx].enablePin, HIGH);
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
