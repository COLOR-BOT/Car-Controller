#include <SoftwareSerial.h>
#include <AccelStepper.h>

SoftwareSerial Bluetooth(10, 38);         // Arduino(RX, TX) - HC-05 Bluetooth (TX, RX)

// Battery Monitoring
#define led 14
int sensorValue = analogRead(A0);

// ACTUATOR 
#define actuatorUp 12
#define actuatorDown 11

// STEPPER MOTORS (Type:driver, STEP, DIR)
AccelStepper LeftBackWheel(1, 42, 43);    // Stepper1
AccelStepper LeftFrontWheel(1, 40, 41);   // Stepper2
AccelStepper RightBackWheel(1, 44, 45);   // Stepper3
AccelStepper RightFrontWheel(1, 46, 47);  // Stepper4

int lbw[50], lfw[50], rbw[50], rfw[50]; // Stepper Motors - STORED Positions / Steps
int wheelSpeed = 1500, dataIn, m, index = 0;

void setup() {
  // Set initial seed values for the steppers
  LeftFrontWheel.setMaxSpeed(3000);
  LeftBackWheel.setMaxSpeed(3000);
  RightFrontWheel.setMaxSpeed(3000);
  RightBackWheel.setMaxSpeed(3000);
  
  Serial.begin(9600);       // Default baud rate of Serial Terminal (for debugging purposes) 
  Bluetooth.begin(9600);    // Default baud rate of Bluetooth module

  delay(20); //  Bluetooth.setTimeout(1);
  pinMode(led, OUTPUT);
  pinMode(actuatorUp, OUTPUT);
  pinMode(actuatorDown, OUTPUT);
}

void loop() {
  // Check for incoming data
  if (Bluetooth.available()) {
    dataIn = Bluetooth.read();  // Read data coming from Bluetooth
     Serial.print(dataIn);      // Print data coming from Bluetooth to Serial Terminal 

    // 0 - 12: Controls wheels 
    if (dataIn == 0)  m = 0;
    if (dataIn == 1)  m = 1;
    if (dataIn == 2)  m = 2;
    if (dataIn == 3)  m = 3;
    if (dataIn == 4)  m = 4;
    if (dataIn == 5)  m = 5;
    if (dataIn == 6)  m = 6;
    if (dataIn == 7)  m = 7;
    if (dataIn == 8)  m = 8;
    if (dataIn == 9)  m = 9;
    if (dataIn == 10) m = 10;
    if (dataIn == 11) m = 11;
    if (dataIn == 12) m = 12;

    // 13 & 14: Controls actuator
    if (dataIn == 13) m = 13;
    if (dataIn == 14) m = 14;
    
    // Set speed
    if (dataIn >= 16) {
      wheelSpeed = dataIn * 10;
      Serial.println(wheelSpeed);
    }
  }
  
  if (m == 0)  stopMoving();
  if (m == 1)  moveLeftForward();
  if (m == 2)  moveForward();
  if (m == 3)  moveRightForward();
  if (m == 4)  moveSidewaysLeft();
  if (m == 5)  moveSidewaysRight();
  if (m == 6)  moveLeftBackward();
  if (m == 7)  moveBackward();
  if (m == 8)  moveRightBackward();
  if (m == 9)  rotateLeft();
  if (m == 10) rotateRight();
 
  if (m == 11) { // If button "SAVE" is pressed
    if (index == 0) {
      LeftBackWheel.setCurrentPosition(0);
      LeftFrontWheel.setCurrentPosition(0);
      RightBackWheel.setCurrentPosition(0);
      RightFrontWheel.setCurrentPosition(0);
    }

    // save position into the array
    lbw[index] = LeftBackWheel.currentPosition();  
    lfw[index] = LeftFrontWheel.currentPosition();
    rbw[index] = RightBackWheel.currentPosition();
    rfw[index] = RightFrontWheel.currentPosition();
    
    index++;  // Increase the array index
    m = 0;
  }
  
  if (m == 12) {
    runSteps();
    
    if (dataIn != 12) {
      stopMoving();
      memset(lbw, 0, sizeof(lbw)); // Clear the array data to 0
      memset(lfw, 0, sizeof(lfw));
      memset(rbw, 0, sizeof(rbw));
      memset(rfw, 0, sizeof(rfw));
      index = 0;  // Index to 0
    }
  }
  
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();

  // Actuator Controller
  if (m == 13) goUp();
  if (m == 14) goDown();

  
  // Monitor the battery voltage
  float voltage = sensorValue * (5.0 / 1023.00) * 3; // Convert the reading values from 5v to suitable 12V i
  //  Serial.println(voltage);
  if (voltage < 11) digitalWrite(led, HIGH); // If voltage is below 11V turn on the LED
  else digitalWrite(led, LOW);
}

  void goUp() {
    digitalWrite(actuatorDown,0);
    digitalWrite(actuatorUp,1);
  }
  
  void goDown() {
    digitalWrite(actuatorUp,0);
    digitalWrite(actuatorDown,1);
  }
  
void moveForward() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(wheelSpeed);
}

void moveBackward() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(-wheelSpeed);
}

void moveSidewaysRight() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(wheelSpeed);
}

void moveSidewaysLeft() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(-wheelSpeed);
}

void rotateLeft() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(wheelSpeed);
}

void rotateRight() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(-wheelSpeed);
}

void moveRightForward() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(wheelSpeed);
}

void moveRightBackward() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(0);
}

void moveLeftForward() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(0);
}

void moveLeftBackward() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(-wheelSpeed);
}

void stopMoving() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(0);
  
  digitalWrite(actuatorUp,0);
  digitalWrite(actuatorDown,0);
}


void runSteps() {
  for (int i = index - 1; i >= 0; i--) { // Run through all steps(index)
    LeftFrontWheel.moveTo(lfw[i]);
    LeftFrontWheel.setSpeed(wheelSpeed);
    LeftBackWheel.moveTo(lbw[i]);
    LeftBackWheel.setSpeed(wheelSpeed);
    RightFrontWheel.moveTo(rfw[i]);
    RightFrontWheel.setSpeed(wheelSpeed);
    RightBackWheel.moveTo(rbw[i]);
    RightBackWheel.setSpeed(wheelSpeed);
    while (LeftBackWheel.currentPosition() != lbw[i] & LeftFrontWheel.currentPosition() != lfw[i] & RightFrontWheel.currentPosition() != rfw[i] & RightBackWheel.currentPosition() != rbw[i]) {
      LeftFrontWheel.runSpeedToPosition();
      LeftBackWheel.runSpeedToPosition();
      RightFrontWheel.runSpeedToPosition();
      RightBackWheel.runSpeedToPosition();
      if (Bluetooth.available() > 0) {      // Check for incomding data
        dataIn = Bluetooth.read();
        if ( dataIn == 15) {           // If button "PAUSE" is pressed
          while (dataIn != 14) {         // Wait until "RUN" is pressed again
            if (Bluetooth.available() > 0) {
              dataIn = Bluetooth.read();
              if ( dataIn == 13) {
                stopMoving();
                break;
              }
            }
          }
        }
        if (dataIn >= 16) {
          wheelSpeed = dataIn * 10;
          dataIn = 14;
        }
        if ( dataIn == 13) {
          break;
        }
      }
    }
  }
  
  // Go back through steps
  for (int i = 1; i <= index - 1; i++) { // Run through all steps(index)
    LeftFrontWheel.moveTo(lfw[i]);
    LeftFrontWheel.setSpeed(wheelSpeed);
    LeftBackWheel.moveTo(lbw[i]);
    LeftBackWheel.setSpeed(wheelSpeed);
    RightFrontWheel.moveTo(rfw[i]);
    RightFrontWheel.setSpeed(wheelSpeed);
    RightBackWheel.moveTo(rbw[i]);
    RightBackWheel.setSpeed(wheelSpeed);
    while (LeftBackWheel.currentPosition() != lbw[i]& LeftFrontWheel.currentPosition() != lfw[i] & RightFrontWheel.currentPosition() != rfw[i] & RightBackWheel.currentPosition() != rbw[i]) {
      LeftFrontWheel.runSpeedToPosition();
      LeftBackWheel.runSpeedToPosition();
      RightFrontWheel.runSpeedToPosition();
      RightBackWheel.runSpeedToPosition();
      //Serial.print("  current: ");
      //Serial.println(LeftBackWheel.currentPosition());
      if (Bluetooth.available() > 0) {      // Check for incomding data
        dataIn = Bluetooth.read();
        if ( dataIn == 15) {           // If button "PAUSE" is pressed
          while (dataIn != 14) {         // Wait until "RUN" is pressed again
            if (Bluetooth.available() > 0) {
              dataIn = Bluetooth.read();
              if ( dataIn == 13) {
                stopMoving();
                break;
              }
            }
          }
        }
        if (dataIn >= 16) {
          wheelSpeed = dataIn * 10;
          dataIn = 14;
        }
        if ( dataIn == 13) {
          //Serial.println("DEKI");
          break;
        }
      }
    }
  }
}
