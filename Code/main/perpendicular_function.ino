const int PERPENDICULAR_CONFIRM_COUNT = 18;  // how many consecutive samples needed

void alignPerpendicularToWall(int targetDistance_mm) {
  int pconfirm_count = 0;
  int prev_lenc = 0;
  int prev_renc = 0;
  // PD constants for rotation (tune carefully)
  const float Kp = 0.6;
  const float Kd = 1.5;

  static float lastError = 0;

  Serial.println("Starting perpendicular alignment (velocity PD)...");

  while (true) {


    // Read raw pulse durations
    int32_t rawLeft = pulseIn(frontLeftTOF, HIGH, 1000000);
    int32_t rawRight = pulseIn(frontRightTOF, HIGH, 1000000);

    // Apply Kalman filter
    float filt_left = kalman1.update((float)rawLeft);
    float filt_right = kalman2.update((float)rawRight);

    // Convert to mm
    int32_t distLeft = (filt_left - 1000) * 3 / 4;
    int32_t distRight = (filt_right - 1000) * 3 / 4;

    // Skip invalid readings
    if (distLeft <= 0 || distRight <= 0) {
      Serial.println("Invalid sensor reading, skipping...");
      delay(50);
      continue;
    }

    // Compute individual errors to target
    int errorLeft = distLeft - targetDistance_mm;
    int errorRight = distRight - targetDistance_mm;

    // Alignment error
    int alignmentError = distLeft - distRight;

    // PD for rotational velocity
    float derivative = alignmentError - lastError;
    lastError = alignmentError;

    float rotationCorrection = Kp * alignmentError + Kd * derivative;

    // Limit maximum rotation correction (important at high speeds)
    rotationCorrection = constrain(rotationCorrection, -30, 30);

    // Base forward speed
    int baseSpeed = 100;  // higher speed possible now

    // Compute individual motor speeds
    int leftSpeed = constrain(baseSpeed + rotationCorrection, 50, 150);
    int rightSpeed = constrain(baseSpeed - rotationCorrection, 50, 150);

    // Control left motor
    if (errorLeft > 10) {
      analogWrite(leftMotor_pin1, leftSpeed);
      analogWrite(leftMotor_pin2, 0);
    } else if (errorLeft < -10) {
      analogWrite(leftMotor_pin1, 0);
      analogWrite(leftMotor_pin2, leftSpeed);
    } else {
      analogWrite(leftMotor_pin1, 0);
      analogWrite(leftMotor_pin2, 0);
    }

    // Control right motor
    if (errorRight > 10) {
      analogWrite(rightMotor_pin1, rightSpeed);
      analogWrite(rightMotor_pin2, 0);
    } else if (errorRight < -10) {
      analogWrite(rightMotor_pin1, 0);
      analogWrite(rightMotor_pin2, rightSpeed);
    } else {
      analogWrite(rightMotor_pin1, 0);
      analogWrite(rightMotor_pin2, 0);
    }

    if (pconfirm_count < PERPENDICULAR_CONFIRM_COUNT ) {
      if (leftEncoder_count == prev_lenc && rightEncoder_count == prev_renc && abs(errorLeft) <= 10 && abs(errorRight) <= 10) {
        pconfirm_count++;

      } else {
        pconfirm_count = 0;
      }
    } else {
      perpendicular_allowed = false;
      break;
    }
    prev_lenc = leftEncoder_count;
    prev_renc = rightEncoder_count;

    // Debug output
    // Serial.print("L: ");
    // Serial.print(distLeft);
    // Serial.print(" R: ");
    // Serial.print(distRight);
    // Serial.print(" Alignment: ");
    // Serial.println(alignmentError);

    delay(50);
  }
}
