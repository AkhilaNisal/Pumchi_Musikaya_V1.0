int32_t readDistance(uint8_t pin) {
  int32_t t = pulseIn(pin, HIGH, 1000000);
  if (t == 0 || t > 1850) return -1;  // Out of range
  int32_t d = (t - 1000) * 3 / 4;
  if (d < 0) d = 0;
  return d;
}

#define TOF_WALL_CONFIRMATION_SC 10
#define TOF_WALL_CONFIRMATION_LC 15
#define TOF_WALL_CONFIRMATION_THRESHOLD 150

bool isFrontWallAvailable() {
  int conf = 0;

  for (int i = 0; i < TOF_WALL_CONFIRMATION_LC; i++) {
   
    int frontLeft =readDistance(frontRightTOF);
    int frontRight = readDistance(frontLeftTOF);
    if (frontRight < TOF_WALL_CONFIRMATION_THRESHOLD && frontLeft < TOF_WALL_CONFIRMATION_THRESHOLD && frontRight != -1 && frontLeft != -1 ) {

      conf++;
      if (conf > TOF_WALL_CONFIRMATION_SC) {
        return true;
      }
    } else {
      if (conf < 0) conf = 0;
      else conf--;
    }
    delay(10);
  }

  return false;
}

bool isLeftWallAvailable() {
  int conf = 0;

  for (int i = 0; i < TOF_WALL_CONFIRMATION_LC; i++) {
    int left = readDistance(leftSensorPin) ;
    if (left < TOF_WALL_CONFIRMATION_THRESHOLD && left != -1) {
      conf++;
      if (conf > TOF_WALL_CONFIRMATION_SC) {
        return true;

      }
    } else {
      if (conf < 0) conf = 0;
      else conf--;
    }
    delay(10);
  }

  return false;
}

bool isRightWallAvailable() {
  int conf = 0;
  for (int i = 0; i < TOF_WALL_CONFIRMATION_LC; i++) {
    int right = readDistance(rightSensorPin);
    if (right < TOF_WALL_CONFIRMATION_THRESHOLD && right != -1) {
      conf++;
      if (conf > TOF_WALL_CONFIRMATION_SC) {
        return true;
      }
    } else {
      if (conf < 0) conf = 0;
      else conf--;
    }
    delay(10);
  }
  return false;
}