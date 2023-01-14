#include <QTRSensors.h>
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

int m1Speed = 0;
int m2Speed = 0;

const int calibrationSpeed = 180;

const int rightRotation = 1;
const int leftRotation = -1;

const int maximumPasses = 4;

// previous pid values
// kp = 8, ki = 0, kd = 2, interval = [-50, 50], frana -150 -> s-a asezat softu' bine 
float kp = 8;
float ki = 0;
float kd = 2;

float p = 0;
float i = 0;
float d = 0;

int error = 0;
int lastError = 0;

const int maxSpeed = 255;
const int minSpeed = -255;

const int baseSpeed = 210;

const int standStillSpeed = 0;

QTRSensors qtr;

const int sensorCount = 6;
int sensorValues[sensorCount];

const uint8_t leftmostSensor = 5;
const uint8_t rightmostSensor = 0;
const int onSensorThreshold = 700;

const int minSensorValue = 0;
const int maxSensorValue = 5000;

const int minSensorValueThreshhold = -50;
const int maxSensorValueThreshhold = 50;

const int sweetSpotSensorValue = 0;
const int warnningSpotSensorValue = 45;
const int reduceMotorSpeed = 80;

const int brakingSpeed = -190;

void setup() {
  Serial.begin(9600);

  // pinMode setup
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // sensor setup
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);

  // waiting for sensor setup
  delay(500);

  // sensor calibration
  calibration();
}

void loop() {

  // calculateing pid values
  pidControl();

  // motors power
  setMotorSpeed(m1Speed, m2Speed);


  //  DEBUGGING
  //  Serial.print("Error: ");
  //  Serial.println(error);
  //  Serial.print("M1 speed: ");
  //  Serial.println(m1Speed);
  //
  //  Serial.print("M2 speed: ");
  //  Serial.println(m2Speed);
  //
  //  delay(250);
}

void calibration() {
  // turn on Arduino's LED to indicate we are in calibration mode
  digitalWrite(LED_BUILTIN, HIGH);

  int passes = 0;
  int state = rightRotation;

  while (passes <= maximumPasses) {
    qtr.calibrate();
    qtr.read(sensorValues);

    // make a right rotation for calibration
    if (state == rightRotation) {
      if (sensorValues[rightmostSensor] < onSensorThreshold) {
        setMotorSpeed(calibrationSpeed, -calibrationSpeed);
      } else {
        state = leftRotation;
      }
    }

    // make a left rotation for calibration
    if (state == leftRotation) {
      if (sensorValues[leftmostSensor] < onSensorThreshold) {
        setMotorSpeed(-calibrationSpeed, calibrationSpeed);
      } else {
        state = rightRotation;
        passes++;
      }
    }
  }

  // turn off Arduino's LED to indicate we left calibration mode
  digitalWrite(LED_BUILTIN, LOW);
}

// calculate PID value based on error, kp, kd, ki, p, i and d.
void pidControl() {
  // error mapping
  float error = map(qtr.readLineBlack(sensorValues), minSensorValue, maxSensorValue, minSensorValueThreshhold, maxSensorValueThreshhold);

  // set PID values
  p = error;
  i = i + error;
  d = error - lastError;
  lastError = error;

  int motorSpeed = kp * p + ki * i + kd * d;  // = error in this case

  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  // in the first if, you substract the error from m1Speed (you add the negative)
  // in the 2nd if you add the error to m2Speed (you substract the negative)
  // it's just the way the values of the sensors and/or motors lined up
  if (error < sweetSpotSensorValue) {
    m1Speed += motorSpeed;
  } 
  else if (error > sweetSpotSensorValue) {
    m2Speed -= motorSpeed;
  }

  // if the error value is at an extreme, than slow down the speed for giving time to recalibrate
  if (error <= -warnningSpotSensorValue) {
    m2Speed -= reduceMotorSpeed;
  } 
  else if (error >= warnningSpotSensorValue) {
    m1Speed -= reduceMotorSpeed;
  }

  // make sure it doesn't go past limits. You can use -255 instead of 0 if calibrated programmed properly.
  // making sure we don't go out of bounds
  // maybe the lower bound should be negative, instead of 0? This of what happens when making a steep turn
  m1Speed = constrain(m1Speed, brakingSpeed, maxSpeed);
  m2Speed = constrain(m2Speed, brakingSpeed, maxSpeed);
}


// each arguments takes values between -255 and 255. The negative values represent the motor speed in reverse.
void setMotorSpeed(int motor1Speed, int motor2Speed) {
  // remove comment if any of the motors are going in reverse
  //  motor1Speed = -motor1Speed;
  //  motor2Speed = -motor2Speed;

  if (motor1Speed == standStillSpeed) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  } 
  else {
    if (motor1Speed > standStillSpeed) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }

    if (motor1Speed < standStillSpeed) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }

  if (motor2Speed == standStillSpeed) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  } 
  else {
    if (motor2Speed > standStillSpeed) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }

    if (motor2Speed < standStillSpeed) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}