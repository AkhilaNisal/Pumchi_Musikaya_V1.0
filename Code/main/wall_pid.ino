void wall_PID() {
  int leftDistance = readDistance(leftSensorPin);
  int rightDistance = readDistance(rightSensorPin);
  // leftDistance = kalman1.update((float)leftDistance);
  // rightDistance = kalman2.update((float)rightDistance);
  /*
    Serial.print(" L: ");
    Serial.print(leftDistance);
    Serial.print(" R: ");
    Serial.println(rightDistance);
*/
  if (leftDistance == -1 || rightDistance == -1 || rightDistance > 300 || leftDistance > 300) {
    // One or both sensors invalid — stop motors
    stopMotors();
    //Serial.println("Sensor error");
    //delay(10);
    currentPID = NO_WALL;
    return;
  }

  // PID calculations
  error_tof = (leftDistance - rightDistance);
  integral_tof += error_tof;
  float derivative = error_tof - lastError_tof;

  float correction = Kp_tof * error_tof + Ki_tof * integral_tof + Kd_tof * derivative;

  lastError_tof = error_tof;

  // Apply correction to motor speeds
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;
  leftSpeed = constrain(leftSpeed, 80, 200);
  rightSpeed = constrain(rightSpeed, 80, 200);


  //   analogWrite(leftMotor_pin1, leftSpeed);
  //   analogWrite(leftMotor_pin2, 0);
  //   analogWrite(rightMotor_pin1,rightSpeed);
  //   analogWrite(rightMotor_pin2, 0);

  motor_forward(leftSpeed, rightSpeed);

  // Debug output
  Serial.print("L: ");
  Serial.print(leftSpeed);
  Serial.print(" R: ");
  Serial.print(rightSpeed);
  Serial.print(" E: ");
  Serial.print(error_tof);
  Serial.print(" Corr: ");
  Serial.println(correction);

  delay(30);  // Adjust as needed  50
}

void right_wall_PID() {
  int rightDistance = readDistance(rightSensorPin);
  // Serial.println(rightDistance);

  if (rightDistance == -1 || rightDistance > 200) {
    // One or both sensors invalid — stop motors
    stopMotors();
    //Serial.println("Sensor error");
    //delay(100);
    currentPID = NO_WALL;

    return;
  }

  error_right_tof = (70 - rightDistance);
  integral_right_tof += error_right_tof;
  float derivative = error_right_tof - lastError_right_tof;

  float correction = Kp_right_tof * error_right_tof + Ki_right_tof * integral_right_tof + Kd_right_tof * derivative;
  // if (correction > 50 )
  //   {
  //       correction = 15;
  //   }

  //   else if (correction < -50)
  //   {
  //       correction = -15;
  //   }
  lastError_right_tof = error_right_tof;

  // Apply correction to motor speeds
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed = constrain(leftSpeed, 80, 200);
  rightSpeed = constrain(rightSpeed, 80, 200);


  // analogWrite(leftMotor_pin1, leftSpeed);
  // analogWrite(leftMotor_pin2, 0);
  // analogWrite(rightMotor_pin1,rightSpeed);
  // analogWrite(rightMotor_pin2, 0);
  motor_forward(leftSpeed, rightSpeed);

  Serial.print("L: ");
  Serial.print(leftSpeed);
  Serial.print(" R: ");
  Serial.print(rightSpeed);
  Serial.print(" E: ");
  Serial.print(error_right_tof);
  Serial.print(" Corr: ");
  Serial.println(correction);

  delay(30);  //50
}


void left_wall_PID() {
  int leftDistance = readDistance(leftSensorPin);

  if (leftDistance == -1 || leftDistance > 200) {
    // One or both sensors invalid — stop motors
    stopMotors();
    //Serial.println("Sensor error");
    //delay(100);
    currentPID = NO_WALL;

    return;
  }

  updateYaw();
  float error = yaw - targetYaw;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  // use  pid constants for left wall pid
  error_left_tof = (80 - leftDistance);
  integral_left_tof += error_left_tof;
  float derivative = error_left_tof - lastError_left_tof;

  float correction = Kp_right_tof * error_left_tof + Ki_right_tof * integral_left_tof + Kd_right_tof * derivative;
  // if (correction > 55 )
  //   {
  //       correction = 15;
  //   }

  //   else if (correction < -55)
  //   {
  //       correction = -15;
  //   }
  lastError_left_tof = error_left_tof;

  // Apply correction to motor speeds
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed = constrain(leftSpeed, 80, 200);
  rightSpeed = constrain(rightSpeed, 80, 200);


  // analogWrite(leftMotor_pin1, leftSpeed);
  // analogWrite(leftMotor_pin2, 0);
  // analogWrite(rightMotor_pin1,rightSpeed);
  // analogWrite(rightMotor_pin2, 0);
  motor_forward(leftSpeed, rightSpeed);


  Serial.print("L: ");
  Serial.print(leftSpeed);
  Serial.print(" R: ");
  Serial.print(rightSpeed);
  Serial.print(" E: ");
  Serial.print(error_left_tof);
  Serial.print(" D: ");
  Serial.print(derivative);
  Serial.print(" Corr: ");
  Serial.println(correction);

  delay(30);  //50
}