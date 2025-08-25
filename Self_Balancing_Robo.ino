#include <Wire.h>
#include <MPU9250.h>

MPU9250 mpu;

// Motor driver pins
int enA = 5;   // Enable pin for motor A
int in1 = 6;
int in2 = 7;
int enB = 9;   // Enable pin for motor B
int in3 = 10;
int in4 = 11;

// PID variables
double setpoint = 0;     // Desired angle (upright = 0°)
double input, output;
double errSum = 0, lastErr = 0;
double Kp = 25.0;   // <-- start tuning here
double Ki = 1.0;
double Kd = 2.0;

// Complementary filter variables
float angle = 0;
unsigned long lastTime;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Init MPU
  if (!mpu.setup(0x68)) {
    Serial.println("MPU connection failed!");
    while (1);
  }
  Serial.println("MPU initialized!");

  // Motor pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  lastTime = millis();
}

void loop() {
  if (mpu.update()) {
    // Complementary filter for angle
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    float accelAngle = atan2(mpu.getAccY(), mpu.getAccZ()) * 180 / PI;
    float gyroRate = mpu.getGyroX(); // rotation around X-axis

    angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accelAngle;

    // PID calculation
    input = angle;
    double error = setpoint - input;
    errSum += error * dt;
    double dErr = (error - lastErr) / dt;

    output = Kp * error + Ki * errSum + Kd * dErr;
    lastErr = error;

    // Drive motors based on PID output
    driveMotors(output);

    // Debugging
    Serial.print("Angle: "); Serial.print(angle);
    Serial.print("\tError: "); Serial.print(error);
    Serial.print("\tPID Output: "); Serial.println(output);
  }
}

// Motor control function
void driveMotors(double pidValue) {
  int motorSpeed = constrain(abs(pidValue), 0, 255);

  if (pidValue > 0) {
    // Tilted forward → move motors forward to correct
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    // Tilted backward → move motors backward to correct
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }

  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}