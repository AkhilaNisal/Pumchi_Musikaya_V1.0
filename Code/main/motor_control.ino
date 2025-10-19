


// Confirmation counters for wall detection
int leftWallConfirm = 0;
int rightWallConfirm = 0;
int doubleWallConfirm = 0;

int enter_perpendicular_confirm = 0;

const int PER_CONFIRM_COUNT = 2;  //2
const int CONFIRM_COUNT = 150;    // how many consecutive samples needed
const int WALL_THRESHOLD = 200;   // mm threshold for side ToFs

const int WAIT_SAMPLE_COUNT = 3;//50


void waitUntilStill() {
  int conf = 0;
  int lprev_enc = leftEncoder_count;
  int rprev_enc = rightEncoder_count;
  while (true) {
    if (lprev_enc == leftEncoder_count && rprev_enc == rightEncoder_count) {
      conf++;
      if (conf > WAIT_SAMPLE_COUNT) {
        break;
      }
    } else {
      conf = (conf > 0) ? conf - 1 : 0;
    }
    lprev_enc = leftEncoder_count;
    rprev_enc = rightEncoder_count;
    delay(5);
  }
}

void motor_forward(int leftSpeed, int rightSpeed){
    analogWrite(leftMotor_pin1, leftSpeed);
    analogWrite(leftMotor_pin2, 0);
    analogWrite(rightMotor_pin1,rightSpeed);
    analogWrite(rightMotor_pin2, 0);
}

void stopMotors() {
  analogWrite(leftMotor_pin1, 255);
  analogWrite(leftMotor_pin2, 255);
  analogWrite(rightMotor_pin1, 255);
  analogWrite(rightMotor_pin2, 255);
  waitUntilStill();
}

void moveOneCellForward(int targetCounts) {

  // Reset encoder targets
  int init_leftEncoder_count = leftEncoder_count;
  int init_rightEncoder_count = rightEncoder_count;
  targetYaw = yaw;

  while (true) {
    // 1. Check if cell
    // Serial.println((leftEncoder_count));
    // Serial.println((rightEncoder_count));
    // Serial.println((leftEncoder_count + rightEncoder_count) / 2);

    if (((leftEncoder_count + rightEncoder_count) / 2) >= targetCounts + ((init_leftEncoder_count + init_rightEncoder_count) / 2)) {
      stopMotors();
      Serial.println("Reached target cell.");
      pidSelected = false;
      break;
    }

    int32_t rawLeft = pulseIn(frontLeftTOF, HIGH, 1000000);
    int32_t rawRight = pulseIn(frontRightTOF, HIGH, 1000000);
    float filt_left = kalman1.update((float)rawLeft);
    float filt_right = kalman2.update((float)rawRight);
    int32_t distLeft = (filt_left - 1000) * 3 / 4;
    int32_t distRight = (filt_right - 1000) * 3 / 4;

    if (distLeft > 0 && distRight > 0 && distLeft < 180 && distRight < 180 && abs(distRight - distLeft) < 50
        && perpendicular_allowed == true) {
      enter_perpendicular_confirm++;
      if (enter_perpendicular_confirm > PER_CONFIRM_COUNT) {
        Serial.println("perpendi///////////////////////////////////cular");
        stopMotors();
        delay(20);
        Serial.println("HALO1");
        int time = millis();
        Serial.println("HALO2");
        Serial.println((millis() - time) / 1000.0);
        alignPerpendicularToWall(60);
        stopMotors();
        delay(10);


        break;
      }
    } else {
      if (enter_perpendicular_confirm > 0) {
        enter_perpendicular_confirm--;
      } else enter_perpendicular_confirm = 0;
    }

    // 2. Select PID at start of cell using side ToFs
    if (!pidSelected) {
      motor_forward(180, 180);
      delay(5);
      int leftDist = readDistance(leftSensorPin);
      int rightDist = readDistance(rightSensorPin);

      if (leftDist != -1 && leftDist < WALL_THRESHOLD && rightDist != -1 && rightDist < WALL_THRESHOLD) {
        currentPID = DOUBLE_WALL;
        Serial.println("Initial PID: DOUBLE_WALL");
      } else if (leftDist != -1 && leftDist < WALL_THRESHOLD) {
        currentPID = LEFT_WALL;
        Serial.println("Initial PID: LEFT_WALL");
      } else if (rightDist != -1 && rightDist < WALL_THRESHOLD) {
        currentPID = RIGHT_WALL;
        Serial.println("Initial PID: RIGHT_WALL");
      } else {
        currentPID = NO_WALL;
        analogWrite(leftMotor_pin2, 0);
        analogWrite(rightMotor_pin2, 0);
        analogWrite(leftMotor_pin1, leftSpeed);
        analogWrite(rightMotor_pin1, rightSpeed);
        Serial.println("Initial PID: NO_WALL (encoder + gyro only)");
      }
      pidSelected = true;
    }

    // 3. Angle ToFs for dynamic PID selection while moving
    int distance1 = sensor1.readRangeContinuousMillimeters();  // right angle
    int distance2 = sensor2.readRangeContinuousMillimeters();  // left angle
    int leftSide = readDistance(leftSensorPin);
    int rightSide = readDistance(rightSensorPin);

    // Reset confirmations each loop
    bool doubleWallDetected = (distance1 < 250 && distance2 < 250);
    bool leftWallDetected = (distance2 < 250);
    bool rightWallDetected = (distance1 < 250);

    // Confirmation logic
    if (doubleWallDetected && leftSide < WALL_THRESHOLD && rightSide < WALL_THRESHOLD && rightSide != -1 && leftSide != -1) {
      doubleWallConfirm++;
      if (doubleWallConfirm >= CONFIRM_COUNT) {
        currentPID = DOUBLE_WALL;
        Serial.println("DOUBLE_WALL (confirmed)");
      }
    } else {
      doubleWallConfirm--;  // reset if not stable
    }

    if (leftWallDetected && leftSide < WALL_THRESHOLD && leftSide != -1) {
      leftWallConfirm++;
      if (leftWallConfirm >= CONFIRM_COUNT) {
        currentPID = LEFT_WALL;
        Serial.println("LEFT_WALL (confirmed)");
      }
    } else {
      leftWallConfirm--;
    }

    if (rightWallDetected && rightSide < WALL_THRESHOLD && rightSide != -1) {
      rightWallConfirm++;
      if (rightWallConfirm >= CONFIRM_COUNT) {
        currentPID = RIGHT_WALL;
        Serial.println("RIGHT_WALL (confirmed)");
      }
    } else {
      rightWallConfirm--;
    }

    // If walls are missing → fallback to encoder PID
    if ((currentPID == DOUBLE_WALL && (leftSide >= WALL_THRESHOLD || rightSide >= WALL_THRESHOLD)) || (currentPID == LEFT_WALL && leftSide >= WALL_THRESHOLD) || (currentPID == RIGHT_WALL && rightSide >= WALL_THRESHOLD)) {
      currentPID = NO_WALL;
      Serial.println("Falling back to NO_WALL (side walls missing)");
    }

    // 4. Call selected wall PID
    switch (currentPID) {
      case DOUBLE_WALL: wall_PID(); break;
      case LEFT_WALL: left_wall_PID(); break;
      case RIGHT_WALL: right_wall_PID(); break;
      case NO_WALL: encoder_PID(); break;  // encoder + gyro only
    }
    //encoder_PID();
  }
}

