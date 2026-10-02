// ============================================================
// ALGAE FARM AUTOMATION CONTROLLER
// Controllino MAXI - Simplified Version
// ============================================================

#include <Controllino.h>
#include <AccelStepper.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Bounce2.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============================================================
// PIN MAPPING - CONTROLLINO MAXI
// ============================================================
// NOTE: Update these pin numbers when you have your final PCB layout

// Stepper Motors - Solar Rotation
#define SOLAR_ROT_STEP   22
#define SOLAR_ROT_DIR    23
#define SOLAR_ROT_ENA    24

// Stepper Motors - Solar Tilt
#define SOLAR_TILT_STEP  25
#define SOLAR_TILT_DIR   26
#define SOLAR_TILT_ENA   27

// Stepper Motors - Algae Frame
#define ALGAE_STEP       28
#define ALGAE_DIR        29
#define ALGAE_ENA        30

// Stepper Motors - Lobe Pump
#define PUMP_STEP        31
#define PUMP_DIR         32
#define PUMP_ENA         33

// Photoresistors (Analog Input)
#define LDR_MAIN         A0
#define LDR_ROTATION     A1
#define LDR_SOLAR_TILT   A2
#define LDR_ALGAE_TILT   A3

// Sensors
#define PH_PIN           A4
#define TEMP_PIN         34
#define FLOW_PIN         35

// Control Buttons
#define START_BUTTON     CONTROLLINO_IN0
#define EMERGENCY_STOP   CONTROLLINO_IN1

// LCD Display
#define LCD_ADDRESS      0x27

// ============================================================
// SENSOR THRESHOLDS
// ============================================================

#define LIGHT_ON_LEVEL     400
#define LIGHT_OFF_LEVEL    250

#define TEMP_SAFE_LOW      30.0
#define TEMP_SAFE_HIGH     35.0
#define TEMP_CRITICAL_LOW  28.0
#define TEMP_CRITICAL_HIGH 37.0

#define PH_SAFE_LOW        9.0
#define PH_SAFE_HIGH       11.0
#define PH_ACTION_LOW      8.5
#define PH_ACTION_HIGH     11.5

#define FLOW_PULSES_PER_LITER 450.0

// ============================================================
// MOTOR SPEEDS
// ============================================================

#define SOLAR_SPEED      150.0
#define ALGAE_SPEED      300.0
#define PUMP_SPEED       450.0

// ============================================================
// TIMING INTERVALS
// ============================================================

#define SENSOR_INTERVAL        500
#define DISPLAY_INTERVAL       1000
#define FLOW_STOP_TIME         2000
#define EMERGENCY_TIMEOUT      30000

// ============================================================
// GLOBAL STATE VARIABLES
// ============================================================

volatile unsigned long flowPulses = 0;

bool systemRunning = false;
bool dayDetected = false;
bool actionRequired = false;
bool criticalFault = false;
bool emergencyMode = false;

float currentPH = 0.0;
float currentTemperature = 0.0;
float currentFlow = 0.0;

unsigned long lastSensorRead = 0;
unsigned long lastDisplayUpdate = 0;

int lightValue = 0;

Bounce startButton = Bounce();
Bounce emergencyStopButton = Bounce();

// ============================================================
// OBJECT DECLARATIONS
// ============================================================

LiquidCrystal_I2C lcd(LCD_ADDRESS, 20, 4);

OneWire oneWire(TEMP_PIN);
DallasTemperature temperatureSensor(&oneWire);

AccelStepper solarRotation(
  AccelStepper::DRIVER,
  SOLAR_ROT_STEP,
  SOLAR_ROT_DIR
);

AccelStepper solarTilt(
  AccelStepper::DRIVER,
  SOLAR_TILT_STEP,
  SOLAR_TILT_DIR
);

AccelStepper algaeTilt(
  AccelStepper::DRIVER,
  ALGAE_STEP,
  ALGAE_DIR
);

AccelStepper pump(
  AccelStepper::DRIVER,
  PUMP_STEP,
  PUMP_DIR
);

// ============================================================
// FLOW SENSOR INTERRUPT
// ============================================================

void flowPulse() {
  flowPulses++;
}

// ============================================================
// MOTOR DRIVER FUNCTIONS
// ============================================================

void enableDrivers() {
  digitalWrite(SOLAR_ROT_ENA, LOW);
  digitalWrite(SOLAR_TILT_ENA, LOW);
  digitalWrite(ALGAE_ENA, LOW);
  digitalWrite(PUMP_ENA, LOW);
}

void disableDrivers() {
  digitalWrite(SOLAR_ROT_ENA, HIGH);
  digitalWrite(SOLAR_TILT_ENA, HIGH);
  digitalWrite(ALGAE_ENA, HIGH);
  digitalWrite(PUMP_ENA, HIGH);
}

void stopAllMotors() {
  solarRotation.stop();
  solarTilt.stop();
  algaeTilt.stop();
  pump.stop();

  solarRotation.setSpeed(0);
  solarTilt.setSpeed(0);
  algaeTilt.setSpeed(0);
  pump.setSpeed(0);
}

// ============================================================
// SENSOR READING FUNCTIONS
// ============================================================

void readLight() {
  lightValue = analogRead(LDR_MAIN);

  if (lightValue >= LIGHT_ON_LEVEL) {
    dayDetected = true;
  }

  if (lightValue <= LIGHT_OFF_LEVEL) {
    dayDetected = false;
  }
}

void readPHAndTemp() {
  int phRaw = analogRead(PH_PIN);
  float phVoltage = phRaw * 5.0 / 1023.0;

  currentPH = 7.0 + ((2.5 - phVoltage) / 0.21);

  temperatureSensor.requestTemperatures();
  currentTemperature = temperatureSensor.getTempCByIndex(0);

  if (currentTemperature == DEVICE_DISCONNECTED_C) {
    currentTemperature = NAN;
  }
}

void readFlow() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastFlowCalculation;

  if (lastFlowCalculation == 0) {
    lastFlowCalculation = now;
    return;
  }

  if (elapsed < 500) {
    return;
  }

  noInterrupts();
  unsigned long pulses = flowPulses;
  flowPulses = 0;
  interrupts();

  float seconds = elapsed / 1000.0;

  if (seconds > 0) {
    currentFlow = (pulses * 60.0) / (FLOW_PULSES_PER_LITER * seconds);
  } else {
    currentFlow = 0.0;
  }

  lastFlowCalculation = now;
}

unsigned long lastFlowCalculation = 0;

void evaluateSystem() {
  actionRequired = false;
  criticalFault = false;

  // Temperature evaluation
  if (isnan(currentTemperature)) {
    criticalFault = true;
  } else {
    if (currentTemperature < TEMP_SAFE_LOW || currentTemperature > TEMP_SAFE_HIGH) {
      actionRequired = true;
    }

    if (currentTemperature < TEMP_CRITICAL_LOW || currentTemperature > TEMP_CRITICAL_HIGH) {
      criticalFault = true;
    }
  }

  // pH evaluation
  if (isnan(currentPH)) {
    criticalFault = true;
  } else {
    if (currentPH < PH_SAFE_LOW || currentPH > PH_SAFE_HIGH) {
      actionRequired = true;
    }

    if (currentPH < PH_ACTION_LOW || currentPH > PH_ACTION_HIGH) {
      criticalFault = true;
    }
  }

  // Flow evaluation
  if (currentFlow < 0.0 || currentFlow > 200.0) {
    criticalFault = true;
  }
}

// ============================================================
// DISPLAY FUNCTIONS
// ============================================================

void updateDisplay() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("T:");
  if (isnan(currentTemperature)) {
    lcd.print("ERR");
  } else {
    lcd.print(currentTemperature, 1);
    lcd.print("C");
  }

  lcd.setCursor(10, 0);
  lcd.print("pH:");
  if (isnan(currentPH)) {
    lcd.print("ERR");
  } else {
    lcd.print(currentPH, 1);
  }

  lcd.setCursor(0, 1);
  lcd.print("Flow:");
  lcd.print(currentFlow, 1);
  lcd.print(" L/m");

  lcd.setCursor(0, 2);
  lcd.print("Light:");
  lcd.print(lightValue);
  lcd.print(dayDetected ? " DAY" : " NIGHT");

  lcd.setCursor(0, 3);

  if (emergencyMode) {
    lcd.print("EMERGENCY ACTIVE");
  } else if (criticalFault) {
    lcd.print("CRITICAL FAULT");
  } else if (actionRequired) {
    lcd.print("ACTION REQUIRED");
  } else if (systemRunning) {
    lcd.print("RUNNING");
  } else {
    lcd.print("STOPPED");
  }
}

// ============================================================
// BUTTON HANDLING
// ============================================================

void handleButtons() {
  startButton.update();
  emergencyStopButton.update();

  // Emergency stop takes priority
  if (emergencyStopButton.fell()) {
    emergencyMode = true;
    systemRunning = false;
  }

  // Start/Stop button only works if not in emergency mode
  if (startButton.fell() && !emergencyMode) {
    if (!criticalFault) {
      systemRunning = !systemRunning;
    }
  }
}

// ============================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================

void runSolarMotors() {
  if (!dayDetected) {
    solarRotation.stop();
    solarTilt.stop();
    return;
  }

  solarRotation.setMaxSpeed(SOLAR_SPEED);
  solarTilt.setMaxSpeed(SOLAR_SPEED);

  solarRotation.setSpeed(SOLAR_SPEED);
  solarTilt.setSpeed(SOLAR_SPEED);

  solarRotation.runSpeed();
  solarTilt.runSpeed();
}

void runAlgaeFrame() {
  algaeTilt.setMaxSpeed(ALGAE_SPEED);
  algaeTilt.setSpeed(ALGAE_SPEED);
  algaeTilt.runSpeed();
}

void runPumpForward() {
  digitalWrite(PUMP_DIR, LOW);

  pump.setMaxSpeed(PUMP_SPEED);
  pump.setSpeed(PUMP_SPEED);
  pump.runSpeed();
}

void runPumpReverse() {
  digitalWrite(PUMP_DIR, HIGH);

  pump.setMaxSpeed(PUMP_SPEED);
  pump.setSpeed(PUMP_SPEED);
  pump.runSpeed();
}

// ============================================================
// EMERGENCY SHUTDOWN SEQUENCE
// ============================================================

void emergencyShutdown() {
  if (emergencyMode == false) {
    return;
  }

  systemRunning = false;
  criticalFault = true;

  enableDrivers();

  // 1. Pump algae back to tank
  digitalWrite(PUMP_DIR, HIGH);
  pump.setMaxSpeed(PUMP_SPEED);
  pump.setSpeed(PUMP_SPEED);

  unsigned long emergencyStart = millis();
  unsigned long lastFlow = millis();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("EMERGENCY SHUTDOWN");
  lcd.setCursor(0, 1);
  lcd.print("Returning algae...");

  while (millis() - emergencyStart < EMERGENCY_TIMEOUT) {
    pump.runSpeed();

    noInterrupts();
    unsigned long pulses = flowPulses;
    flowPulses = 0;
    interrupts();

    if (pulses > 0) {
      lastFlow = millis();
    }

    // No flow for 2 seconds means water is back in tank
    if (millis() - lastFlow >= FLOW_STOP_TIME) {
      break;
    }

    delay(10);
  }

  // 2. Stop pump
  pump.setSpeed(0);
  pump.stop();

  // 3. Return motors to safe position
  solarRotation.setMaxSpeed(SOLAR_SPEED);
  solarTilt.setMaxSpeed(SOLAR_SPEED);
  algaeTilt.setMaxSpeed(ALGAE_SPEED);

  solarRotation.moveTo(2500);
  solarTilt.moveTo(2500);
  algaeTilt.moveTo(2500);

  lcd.setCursor(0, 2);
  lcd.print("Moving to safe pos...");

  while (solarRotation.distanceToGo() != 0 ||
         solarTilt.distanceToGo() != 0 ||
         algaeTilt.distanceToGo() != 0) {
    solarRotation.run();
    solarTilt.run();
    algaeTilt.run();
  }

  // 4. Disable drivers
  stopAllMotors();
  disableDrivers();

  // 5. Display emergency status
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("EMERGENCY STOP");
  lcd.setCursor(0, 1);
  lcd.print("All motors halted");
  lcd.setCursor(0, 2);
  lcd.print("Manual reset needed");
  lcd.setCursor(0, 3);
  lcd.print("Contact supervisor");
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  // I2C and LCD
  Wire.begin();
  lcd.init();
  lcd.backlight();

  // Buttons
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(EMERGENCY_STOP, INPUT_PULLUP);
  startButton.attach(START_BUTTON);
  startButton.interval(25);
  emergencyStopButton.attach(EMERGENCY_STOP);
  emergencyStopButton.interval(25);

  // Stepper driver enable pins
  pinMode(SOLAR_ROT_ENA, OUTPUT);
  pinMode(SOLAR_TILT_ENA, OUTPUT);
  pinMode(ALGAE_ENA, OUTPUT);
  pinMode(PUMP_ENA, OUTPUT);

  // Set driver enables OFF initially
  digitalWrite(SOLAR_ROT_ENA, HIGH);
  digitalWrite(SOLAR_TILT_ENA, HIGH);
  digitalWrite(ALGAE_ENA, HIGH);
  digitalWrite(PUMP_ENA, HIGH);

  // Stepper outputs
  pinMode(SOLAR_ROT_STEP, OUTPUT);
  pinMode(SOLAR_ROT_DIR, OUTPUT);
  pinMode(SOLAR_TILT_STEP, OUTPUT);
  pinMode(SOLAR_TILT_DIR, OUTPUT);
  pinMode(ALGAE_STEP, OUTPUT);
  pinMode(ALGAE_DIR, OUTPUT);
  pinMode(PUMP_STEP, OUTPUT);
  pinMode(PUMP_DIR, OUTPUT);

  // Flow sensor interrupt
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowPulse, FALLING);

  // Temperature sensor
  temperatureSensor.begin();

  // Initial state
  stopAllMotors();
  disableDrivers();

  // Display startup
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Algae Farm");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  delay(1000);
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  handleButtons();

  // Read sensors
  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = millis();

    readLight();
    readPHAndTemp();
    readFlow();
    evaluateSystem();
  }

  // Update display
  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = millis();
    updateDisplay();
  }

  // Emergency stop activated
  if (emergencyMode) {
    emergencyShutdown();
  }

  // Critical fault triggers emergency
  if (criticalFault && !emergencyMode) {
    emergencyMode = true;
  }

  // Normal operation
  if (systemRunning && !criticalFault && !emergencyMode) {
    enableDrivers();
    runSolarMotors();
    runAlgaeFrame();

    if (dayDetected) {
      runPumpForward();
    } else {
      runPumpReverse();
    }
  } else {
    stopAllMotors();
    if (!emergencyMode) {
      disableDrivers();
    }
  }

  delay(10);
}
