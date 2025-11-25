// --- Pin connections ---
const int dirPin = 2;  
const int stepPin = 3;  
const int stepsPerRevolution = 275;  // 1 full rev at full step

// timing for each pulse (1000 + 1000 µs = 500 steps/s)
const int pulseDelay = 1000;

// state
bool isOpen = false;

void setup() {
  Serial.begin(115200);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);

  Serial.println("Stepper ready (manual pulse mode)");
}

void loop() {
  // ------------------- Handle Serial Commands -------------------
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'c') {
      Serial.println("Opening: 1 full CW rotation");
      rotateCW(stepsPerRevolution);
      isOpen = true;
    } 
    
    else if (cmd == 'o') {
      Serial.println("Closing: 1 full CCW rotation");
      rotateCCW(stepsPerRevolution);
      isOpen = false;
    }
    
    else {
      Serial.print("Unknown command: ");
      Serial.println(cmd);
    }
  }
}

// ------------------- Movement Functions -------------------

void rotateCW(int steps) {
  digitalWrite(dirPin, HIGH);   // clockwise
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelay);
  }
}

void rotateCCW(int steps) {
  digitalWrite(dirPin, LOW);    // anticlockwise
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelay);
  }
}
