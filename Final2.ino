#include <Servo.h>

const int inputPin = A0;      // Analog input pin for voltage measurement
const int servoPin = 9;       // Digital pin for servo control
const int stopVoltage = 500;  // Voltage threshold for stopping (0.5V)
const int startVoltage = 200; // Voltage threshold for starting (2V)
const unsigned long validationDuration = 500; // Validation duration in milliseconds

Servo servoMotor;
int remainingTime = 0;       // Remaining angle to reverse (in degrees)
unsigned long validationStartTime = 0; // Start time for voltage validation
unsigned long elapsedTime = 0;
unsigned long elapsedTimeOut = 0;
bool powerOn = false;

void setup() {
  pinMode(inputPin, INPUT);   // Set input pin as an input
  servoMotor.attach(servoPin);// Attach servo to the designated pin
  Serial.begin(9600);
  servoMotor.writeMicroseconds(1500); // Set initial position to stop
}

void loop() {
  int voltage = analogRead(inputPin);
  bool justOn = false;

  // Check if the voltage is above the start threshold for validation duration
  if (isPowerOn() && validateVoltage(true)) {
    justOn = true;
    servoMotor.write(100); // Spin the servo in one direction
    powerTimer();
    while (voltage > stopVoltage){
      voltage = analogRead(inputPin);
    }
    elapsedTime = powerTimer();
    Serial.print("Elapsed Time:");
    Serial.println(elapsedTime);
  }
  // Check if the voltage is below the stop threshold for validation duration
  else if (voltage < stopVoltage && justOn && validateVoltage(false)) {
    justOn = false;
    servoMotor.write(89); // Spin the servo in the other direction
    remainingTime = elapsedTime;
    powerTimer();

    while (remainingTime > 0){
      elapsedTimeOut = powerTimer();
      remainingTime = elapsedTime - elapsedTimeOut;
      Serial.print("Remaining Time:");
      Serial.println(remainingTime);
    }
  }
  // If voltage is not within the validation duration, stop the motor
  else {
    servoMotor.writeMicroseconds(1500); // Stop the servo
  }
}

bool validateVoltage(bool aboveThreshold) {
  unsigned long currentTime = millis();

  // Check if the validation duration has passed
  if (currentTime - validationStartTime >= validationDuration) {
    // Reset the validation start time
    validationStartTime = 0;
    return true;
  }

  // Check if the voltage consistently remains above or below the threshold
  if ((aboveThreshold && analogRead(inputPin) >= startVoltage) ||
      (!aboveThreshold && analogRead(inputPin) < stopVoltage)) {
    // If voltage is consistent, start or stop validation duration
    if (validationStartTime == 0) {
      validationStartTime = currentTime;
    }
  } else {
    // Reset the validation start time
    validationStartTime = 0;
  }

  return false;
}

bool isPowerOn() {
  int voltage = analogRead(inputPin);
  return voltage >= startVoltage;
}

int powerTimer() {
  static unsigned long startTime = 0;
  static bool timerStarted = false;

  if (isPowerOn()) {
    if (!timerStarted) {
      startTime = millis();
      timerStarted = true;
    }
  } else {
    if (timerStarted) {
      unsigned long elapsedTime = millis() - startTime;
      Serial.print("Timer stopped. Elapsed time: ");
      Serial.print(elapsedTime);
      Serial.println(" ms");
      timerStarted = false;
      return elapsedTime;
    }
  }
}