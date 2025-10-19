float gyroZ_offset = 24.9;

void calibrateGyro() {
  long sum = 0;
  int samples = 2000;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    while (Wire.available() < 2)
      ;

    int16_t gyroZ_raw = (Wire.read() << 8) | Wire.read();
    sum += gyroZ_raw;

    delay(10);
  }

  gyroZ_offset = (float)sum / samples;
  Serial.print("Gyro Z offset = ");
  Serial.println(gyroZ_offset);
}


void updateYaw() {
  int16_t gyroZ_raw;
  float gyroZ_dps;
  float dt;

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true);

  if (Wire.available() >= 2) {
    gyroZ_raw = (Wire.read() << 8) | Wire.read();
    gyroZ_dps = (gyroZ_raw - gyroZ_offset) / 131.0;  // subtract offset

    unsigned long now = millis();
    dt = (now - lastTime) / 1000.0;
    lastTime = now;

    yaw += gyroZ_dps * dt;

    if (yaw > 180.0) yaw -= 360.0;
    if (yaw < -180.0) yaw += 360.0;
  }
}



float targetYaw = yaw;

void moveForward_GYRO_PID() {

  updateYaw();
  float error = yaw - targetYaw;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  gyro_integral += error;
  gyro_integral = constrain(gyro_integral, -100, 100);

  float derivative = error - gyro_lastError;
  gyro_lastError = error;

  float correction = gyro_Kp * error + gyro_Ki * gyro_integral + gyro_Kd * derivative;



  // Adjust speeds for straight-line correction
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  // Limit motor speed to 0–255
  leftSpeed = constrain(leftSpeed, 100, 140);
  rightSpeed = constrain(rightSpeed, 100, 140);

  // Apply motor speeds (forward direction)
  // analogWrite(leftMotor_pin1, leftSpeed);
  // analogWrite(leftMotor_pin2, 0);
  // analogWrite(rightMotor_pin1, rightSpeed);
  // analogWrite(rightMotor_pin2, 0);


  Serial.print("Yaw: ");
  Serial.print(yaw, 2);
  Serial.print("°, Error: ");
  Serial.print(error, 2);
  Serial.print(", L: ");
  Serial.print(leftSpeed);
  Serial.print(", R: ");
  Serial.println(rightSpeed);

  delay(2);  // PID loop rate
}

void turnLeft_PID(float targetAngle) {
  float turn_Kp = 0.5;
  float turn_Ki = 0.0001;
  float turn_Kd = 0.1;

  float turn_integral = 0;
  float turn_lastError = 0;
  int zeroCrossings = 0;
  updateYaw();

  float startYaw = yaw;
  float error, derivative, outputSpeed;


  float prevError = 0;

  while (true) {
    updateYaw();

    error = targetAngle - (yaw - startYaw);
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    turn_integral += error;
    if (turn_integral > 100) turn_integral = 100;
    else if (turn_integral < -100) turn_integral = -100;

    derivative = error - turn_lastError;
    turn_lastError = error;

    outputSpeed = turn_Kp * error + turn_Ki * turn_integral + turn_Kd * derivative;

    float speed;

    if (abs(error) > 50) {
      speed = 120;
    } else {
      speed = constrain(abs(outputSpeed), 100, 120);
    }

    if (outputSpeed < 0) {
      analogWrite(leftMotor_pin1, speed);
      analogWrite(leftMotor_pin2, 0);
      analogWrite(rightMotor_pin1, 0);
      analogWrite(rightMotor_pin2, speed);
    } else {
      analogWrite(leftMotor_pin1, 0);
      analogWrite(leftMotor_pin2, speed);
      analogWrite(rightMotor_pin1, speed);
      analogWrite(rightMotor_pin2, 0);
    }

    float turnedAngle = yaw - startYaw;
    if (turnedAngle > 180) turnedAngle -= 360;
    if (turnedAngle < -180) turnedAngle += 360;

    //Serial.print("Turning... Current angle: ");
    //Serial.print(turnedAngle, 2);
    // Serial.print(speed);
    // Serial.print("°, Error: ");
    //  Serial.println(error, 2);

    if (prevError * error < 0) {
      zeroCrossings++;
      Serial.print("Oscillation detected: ");
      Serial.println(zeroCrossings);
    }

    prevError = error;

    if (zeroCrossings >= 5) {
      stopMotors();
      setPWMBurst();
      leftEncoder_count = 0;
      rightEncoder_count = 1;
      Serial.println("Turn completed.");
      perpendicular_allowed = true;
      break;
    }

    delay(10);
  }
}

void setPWMBurst() {
  int lprev_enc = leftEncoder_count;
  int rprev_enc = rightEncoder_count;
  int i = 30;
  while (true) {
    i += 1;
    motor_forward(constrain(i, 30, 255), constrain(i, 30, 255));
    delay(5);
    if (leftEncoder_count > lprev_enc + 15 && rightEncoder_count > rprev_enc + 15) {
      break;
    }
  }
}