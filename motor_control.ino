// Motor Driver Pins (TB6612FNG) have fun with the wires
const int AIN1 = 12;
const int AIN2 = 13;
const int PWMA = 5;
const int BIN1 = 8;
const int BIN2 = 9;
const int PWMB = 6;
const int STBY = 10;

// Encoder Pins (Yellow = Phase A, Green = Phase B)
const int ENC_A_A = 2; // Must be 2 or 3 for interrupts
const int ENC_A_B = 7; 

const int ENC_B_A = 3; // Must be 2 or 3 for interrupts yellow
const int ENC_B_B = 4; 

// Volatile variables for use in Interrupts
volatile long A_pulseCount = 0;
long A_lastPrintedCount = 0;
volatile long B_pulseCount = 0;
long B_lastPrintedCount = 0;

// Encoder Interrupt Service Routine (ISR)
// This logic determines direction based on the "Quadrature" signal
void A_ISR() {
  if (digitalRead(ENC_A_B) == LOW) {
    A_pulseCount--;
  } else {
    A_pulseCount++;
  }
}



void B_ISR() {
  if (digitalRead(ENC_B_B) == LOW) {
    B_pulseCount++;
  } else {
    B_pulseCount--;
  }
}

void setup() {
  Serial.begin(115200); // Using 115200 for faster data flow

  // Motor Pin Setup
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  // Start the driver
  digitalWrite(STBY, HIGH);

  // Encoder Pin Setup
  pinMode(ENC_A_A, INPUT_PULLUP);
  pinMode(ENC_A_B, INPUT_PULLUP);
  pinMode(ENC_B_A, INPUT_PULLUP);
  pinMode(ENC_B_B, INPUT_PULLUP);

  // Attach interrupt to ENC_A (Yellow wire)
  attachInterrupt(digitalPinToInterrupt(ENC_A_A), A_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B_A), B_ISR, RISING);
  // Serial.println("System Online. Type -255 to 255:");
}

void loop() {
  // Handle Keyboard Commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if(cmd=='m'){
      int cmd_A = Serial.parseInt();
      int cmd_B = Serial.parseInt();
      while(Serial.available() > 0) {
        Serial.read(); // "Flush" the newline/enter characters
      }

      if (cmd_A >0) {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      analogWrite(PWMA, cmd_A);
      // Serial.print(">> Forward A");
      // Serial.println(cmd_A);
    } 
    else if (cmd_A <0) {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      analogWrite(PWMA, -cmd_A);
      // Serial.println(">> Reverse A");
      // Serial.println(-cmd_A);
    } 
    else if (cmd_A == 0) {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      analogWrite(PWMA, 0);
      // Serial.println(">> Stop A");
    }
    if (cmd_B >0) {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      analogWrite(PWMB, cmd_B);
      // Serial.print(">> Forward B");
      // Serial.println(cmd_B);
    } 
    else if (cmd_B <0) {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      analogWrite(PWMB, -cmd_B);
      // Serial.println(">> Reverse B");
      // Serial.println(-cmd_B);
    } 
    else if (cmd_B == 0) {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
      analogWrite(PWMB, 0);
      // Serial.println(">> Stop B");
    }

    }
    else if (cmd=='e'){
      // Serial.print("Position: ");
      Serial.print(-A_pulseCount);
      Serial.print(" ");
      Serial.println(-B_pulseCount);
    }
    

    
  }

  //Print encoder position only when it changes
  // if (A_pulseCount != A_lastPrintedCount || B_pulseCount != B_lastPrintedCount) {
  //   Serial.print("Position: ");
  //   Serial.print(A_pulseCount);
  //   Serial.print(" ");
  //   Serial.println(B_pulseCount);
  //   A_lastPrintedCount = A_pulseCount;
  //   B_lastPrintedCount = B_pulseCount;
  // }
}