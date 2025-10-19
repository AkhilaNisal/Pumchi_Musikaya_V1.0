void handleLeftEncoder() {
  bool A = digitalRead(leftEncoderPinA);
  bool B = digitalRead(leftEncoderPinB);
  leftEncoder_count += (A == B) ? 1 : -1;
}

void handleRightEncoder() {
  bool A = digitalRead(rightEncoderPinA);
  bool B = digitalRead(rightEncoderPinB);
  rightEncoder_count += (A == B) ? -1 : 1;
}

void encoder_PID(){
  if ((leftEncoder_count - lastLeftCorrectionCount >= correctionStep) || (rightEncoder_count - lastRightCorrectionCount >= correctionStep)) {
      int leftDelta  = leftEncoder_count - lastLeftCorrectionLeft;
      int rightDelta = rightEncoder_count - lastLeftCorrectionRight;


      lastLeftCorrectionLeft  = leftEncoder_count;
      lastLeftCorrectionRight = rightEncoder_count;

      int error = leftDelta - rightDelta;
      integral += error;
      int derivative = error - lastError;

      int correction = Kp * error + Ki * integral + Kd * derivative;

      lastError = error;
    
      leftSpeed  = constrain(baseSpeed + correction, 80, 200);
      rightSpeed = constrain(baseSpeed - correction, 80, 200);

      analogWrite(leftMotor_pin2, 0);
      analogWrite(rightMotor_pin2, 0);
      analogWrite(leftMotor_pin1, leftSpeed);
      analogWrite(rightMotor_pin1, rightSpeed);
      // motor_forward(leftSpeed, rightSpeed);


      // Serial.print("L: ");
      // Serial.print(leftSpeed);
      // Serial.print(" R: ");
      // Serial.println(rightSpeed);
  }
}


