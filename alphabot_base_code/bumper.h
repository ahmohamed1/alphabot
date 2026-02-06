// bumper sensords
#define LEFT_BUMPER_PIN 7
#define RIGHT_BUMPER_PIN 8

enum BumperStatus {
  NO_OBJECT,   // 0: No sensor pressed
  LEFT_SIDE,   // 1: Left sensor pressed
  RIGHT_SIDE,  // 2: Right sensor pressed
  CENTER       // 3: Both sensors pressed
};

void setupBumper() {
  pinMode(LEFT_BUMPER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUMPER_PIN, INPUT_PULLUP);
}

int checkBumper() {
  bool leftPressed  = (digitalRead(LEFT_BUMPER_PIN)  == LOW);
  bool rightPressed = (digitalRead(RIGHT_BUMPER_PIN) == LOW);

  if (!leftPressed && !rightPressed) {
    return NO_OBJECT;
  } 
  else if (leftPressed && !rightPressed) {
    return LEFT_SIDE;
  } 
  else if (!leftPressed && rightPressed) {
    return RIGHT_SIDE;
  } 
  else {
    return CENTER;
  }
}
