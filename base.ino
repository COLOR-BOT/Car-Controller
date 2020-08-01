#include <SoftwareSerial.h>

// Arduino(RX, TX) - HC-05 Bluetooth (TX, RX)
// BT1 - app | BT2 - top mechanism
SoftwareSerial Bluetooth1(A8, 38);      
SoftwareSerial Bluetooth2(A9, 39);      

// MOTORS 
/* 
 * 0 & 1 connected to same dual driver 
 * 2 & 3 connected to same dual driver
 * 
 * 0 - Right Front Wheel
 * 1 - Left Front Wheel
 * 2 - Right Back Wheel
 * 3 - Left Back Wheel
 */
#define BRAKEVCC 0
#define CW   1        // Clockwise Direction
#define CCW  2        // Counter-Clockwise Direction
#define BRAKEGND 3
#define CS_THRESHOLD 145

int inApin[4] = {7, 4, 2, 11};    // In-A: Clockwise input pins
int inBpin[4] = {8, 9, 13, 12};   // In-B: Counter-clockwise input pins
int PWMpin[4] = {5, 6, 10, 3};    // PWM input

// Speed
#define PWM_MAX  25
#define PWM_HALF 100

// ACTUATOR 
#define actuatorUp 30
#define actuatorDown 31

// Ultrasonic Sensors
/*
 * 0 Facing Wall (LEFT)
 * 1 - Front
 * 2 - Back
 */
const int pingPin0 = 47, pingPin1 = 43, pingPin2 = 45;  // Trigger Pin of Ultrasonic Sensor
const int echoPin0 = 46, echoPin1 = 42, echoPin2 = 44;  // Echo Pin of Ultrasonic Sensor

void setup() {
  Serial.begin(9600);        // Default baud rate of Serial Terminal (for debugging purposes) 
  while (!Serial) Serial.print("Serial NOT initialized\n");     // wait for serial port to connect. Needed for native USB port only
  Serial.print("Serial initialized\n");

  Bluetooth1.begin(9600);    // Default baud rate of Bluetooth module
  while (!Bluetooth1) Serial.print("BT 1 NOT initialized\n");   // wait for serial port to connect.
  Serial.print("Bluetooth 1 initialized\n");
  
  Bluetooth2.begin(9600);    // Default baud rate of Bluetooth module
  while (!Bluetooth2) Serial.print("BT 2 NOT initialized\n");   // wait for serial port to connect.
  Serial.print("Bluetooth 2 initialized\n");
    
  pinMode(actuatorUp, OUTPUT);
  pinMode(actuatorDown, OUTPUT);

  // Initialize digital pins as outputs in a loop
  for (int i = 0; i < 4; i++) {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(PWMpin[i], OUTPUT);

    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
    
  }
  
  Bluetooth1.listen();
  Serial.print("Set-Up completed\n\n");  
}

int lbw[50], lfw[50], rbw[50], rfw[50]; // Stepper Motors - STORED Positions / Steps
int wheelSpeed = PWM_MAX, dataIn, m, dataOut, index = 0;
String color = "", state = "";
long duration0, inches0, cm0,
     duration1, inches1, cm1,
     duration2, inches2, cm2;

void loop() {  
  dataIn = Bluetooth1.read();         // Read data coming from Bluetooth
   
  // Check for incoming data 
  if (dataIn == 0) {
    Serial.print("0 - STOP Moving\n");
    stopMoving();
  }
  else if (dataIn == 1) {
    Serial.print("1 - Left Forward\n");      
    moveLeftForward();
  }       
  else if (dataIn == 2) {
    Serial.print("2 - Forward\n");      
    moveForward();
  }
  else if (dataIn == 3) {
    Serial.print("3 - Right Forward\n");      
    moveRightForward();
  } 
  else if (dataIn == 4) {
    Serial.print("4 - Sideways Left\n");      
    moveSidewaysLeft();
  } 
  else if (dataIn == 5) {
    Serial.print("5 - Sideways Right\n");      
    moveSidewaysRight();
  } 
  else if (dataIn == 6) {
    Serial.print("6 - Left Forward\n");      
    moveLeftForward();
  } 
  else if (dataIn == 7) {
    Serial.print("7 - Backward\n");      
    moveBackward();
  } 
  else if (dataIn == 8) {
    Serial.print("8 - Right Backwards\n");      
    moveRightBackward();
  } 
  else if (dataIn == 9) {
    Serial.print("9 - Rotate Left\n");      
    rotateLeft();
  } 
  else if (dataIn == 10) {
    Serial.print("10 - Rotate Right\n");      
    rotateRight();
  } 

  // Actuator Controller
  else if (dataIn == 11) {
    Serial.print("11 - Actuator Up\n");      
    goUp();
  } 
  else if (dataIn == 12) {
    Serial.print("12 - Actuator Down\n");      
    goDown();
  }

  // Paint Controllers
  else if (dataIn == 13) {
    Serial.print("13 - Paint Left Up\n");      
    paintLeftUp();
  } 
  else if (dataIn == 14) {
    Serial.print("14 - Paint Up\n");      
    paintUp();
  } 
  else if (dataIn == 15) {
    Serial.print("15 - Paint Right Up\n");      
    paintRightUp();
  } 
  else if (dataIn == 16) {
    Serial.print("16 - Paint Left\n");      
    paintLeft();
  } 
  else if (dataIn == 17) {
    Serial.print("17 - Paint Right\n");      
    paintRight();
  } 
  else if (dataIn == 18) {
    Serial.print("18 - Paint Left Down\n");      
    paintLeftDown();
  } 
  else if (dataIn == 19) {
    Serial.print("19 - Paint Down\n");      
    paintDown();
  } 
  else if (dataIn == 20) {
    Serial.print("20 - Paint Right Down\n");      
    paintRightDown();
  } 
  
  else if (dataIn == 21) {
    Serial.print("21 - cyan\n");      
    color = "cyan";
  }
  else if (dataIn == 22) {
    Serial.print("22 - yellow\n");      
    color = "yellow";
  }
  else if (dataIn == 23) {
    Serial.print("23 - magenta\n");      
    color = "magenta";  
  }
  else if (dataIn == 24) {
    Serial.print("24 - black\n");      
    color = "black";
  }
  
  if (dataIn > 25) {
    wheelSpeed = dataIn;
    Serial.println ("Wheel Speed: ");
    Serial.println(wheelSpeed);
    Serial.print("\n");
  } 

  if (color == "cyan" & dataOut != 1 & dataOut != 0) {
    Serial.println ("Old Data Out: ");
    Serial.println(dataOut);
    Serial.print("\n");
    dataOut = 1;
    Serial.println ("New Data Out: ");
    Serial.println(dataOut);
    Serial.print("\n");
    
    Bluetooth2.write(index);
    Bluetooth2.write(dataOut);
    Bluetooth1.listen();
  }
  else if (color == "magenta" & dataOut != 2 & dataOut != 0) {
    Serial.println ("Old Data Out: ");
    Serial.println(dataOut);
    Serial.print("\n");
    dataOut = 2;
    Serial.println ("New Data Out: ");
    Serial.println(dataOut);
    Serial.print("\n");
    
    Bluetooth2.write(index);
    Bluetooth2.write(dataOut);
    Bluetooth1.listen();
  }
  if (color == "yellow" & dataOut != 3 & dataOut != 0) {
    Serial.println ("Old Data Out: ");
    Serial.println(dataOut);
    Serial.print("\n");
    dataOut = 3;
    Serial.println ("New Data Out: ");
    Serial.println(dataOut);
    Serial.print("\n");

    Bluetooth2.write(index);
    Bluetooth2.write(dataOut);
    Bluetooth1.listen();
  }
  if (color == "black" & dataOut != 4 & dataOut != 0) {
    Serial.println ("Old Data Out: ");
    Serial.println(dataOut);
    Serial.print("\n");
    dataOut = 4;
    
    Serial.println ("New Data Out: ");
    Serial.println(dataOut);
    Serial.print("\n");

    Bluetooth2.write(index);
    Bluetooth2.write(dataOut);
    Bluetooth1.listen();
  }

  // Ultrasonic Sensor 0 (Facing Wall)
  pinMode(pingPin0, OUTPUT);
  digitalWrite(pingPin0, LOW);
  
  delayMicroseconds(2);
  digitalWrite(pingPin0, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(pingPin0, LOW);
  pinMode(echoPin0, INPUT);

  duration0 = pulseIn(echoPin0, HIGH);
  inches0 = microsecondsToInches(duration0);
  cm0 = microsecondsToCentimeters(duration0);

  // Ultrasonic Sensor 1 (FRONT)
  pinMode(pingPin1, OUTPUT);
  digitalWrite(pingPin1, LOW);
  
  delayMicroseconds(2);
  digitalWrite(pingPin1, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(pingPin1, LOW);
  pinMode(echoPin1, INPUT);

  duration1 = pulseIn(echoPin1, HIGH);
  inches1 = microsecondsToInches(duration1);
  cm1 = microsecondsToCentimeters(duration1);

  // Ultrasonic Sensor 2 (BACK)
  pinMode(pingPin2, OUTPUT);
  digitalWrite(pingPin2, LOW);
  
  delayMicroseconds(2);
  digitalWrite(pingPin2, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(pingPin2, LOW);
  pinMode(echoPin2, INPUT);

  duration2 = pulseIn(echoPin2, HIGH);
  inches2 = microsecondsToInches(duration2);
  cm2 = microsecondsToCentimeters(duration2);

  if(inches0 < 12 || inches1 < 12 || inches2 < 12) stopMoving();
}


void moveForward() {
  motorGo(0, CCW, wheelSpeed);      // 0 - Right Front Wheel
  motorGo(1, CW, wheelSpeed);       // 1 - Left Front Wheel
  motorGo(2, CCW, wheelSpeed);      // 2 - Right Back Wheel
  motorGo(3, CW, wheelSpeed);       // 3 - Left Back Wheel
}

void moveBackward() {
  motorGo(0, CW, wheelSpeed);       // 0 - Right Front Wheel
  motorGo(1, CCW, wheelSpeed);      // 1 - Left Front Wheel
  motorGo(2, CW, wheelSpeed);       // 2 - Right Back Wheel
  motorGo(3, CCW, wheelSpeed);      // 3 - Left Back Wheel
}

void moveSidewaysRight() {
   motorGo(0, CW, wheelSpeed);    // 0 - Right Front Wheel
   motorGo(1, CW, wheelSpeed);    // 1 - Left Front Wheel
   motorGo(2, CW, wheelSpeed);    // 2 - Right Back Wheel
   motorGo(3, CW, wheelSpeed);    // 3 - Left Back Wheel
}

void moveSidewaysLeft() {
   motorGo(0, CCW, wheelSpeed);   // 0 - Right Front Wheel
   motorGo(1, CCW, wheelSpeed);   // 1 - Left Front Wheel
   motorGo(2, CCW, wheelSpeed);   // 2 - Right Back Wheel
   motorGo(3, CCW, wheelSpeed);   // 3 - Left Back Wheel
}

void rotateRight() {
   motorGo(0, CW, wheelSpeed);    // 0 - Right Front Wheel
   motorGo(1, CCW, wheelSpeed);   // 1 - Left Front Wheel
   motorGo(2, CCW, wheelSpeed);   // 2 - Right Back Wheel
   motorGo(3, CW, wheelSpeed);    // 3 - Left Back Wheel
}

void rotateLeft() {
   motorGo(0, CCW, wheelSpeed);   // 0 - Right Front Wheel
   motorGo(1, CW, wheelSpeed);    // 1 - Left Front Wheel
   motorGo(2, CW, wheelSpeed);    // 2 - Right Back Wheel
   motorGo(3, CCW, wheelSpeed);   // 3 - Left Back Wheel
}

void moveRightForward() {
   stopOneMotor(0);               // 0 - Right Front Wheel
   motorGo(1, CW, wheelSpeed);    // 1 - Left Front Wheel
   motorGo(2, CCW, wheelSpeed);   // 2 - Right Back Wheel
   stopOneMotor(3);               // 3 - Left Back Wheel
}

void moveRightBackward() {
   motorGo(0, CW, wheelSpeed);    // 0 - Right Front Wheel
   stopOneMotor(1);               // 1 - Left Front Wheel
   stopOneMotor(2);               // 2 - Right Back Wheel
   motorGo(3, CCW, wheelSpeed);   // 3 - Left Back Wheel
}

void moveLeftForward() {
   motorGo(0, CCW, wheelSpeed);   // 0 - Right Front Wheel
   stopOneMotor(1);               // 1 - Left Front Wheel
   stopOneMotor(2);               // 2 - Right Back Wheel
   motorGo(3, CW, wheelSpeed);    // 3 - Left Back Wheel
}

void moveLeftBackward() {
   stopOneMotor(0);               // 0 - Right Front Wheel
   motorGo(1, CCW, wheelSpeed);   // 1 - Left Front Wheel
   motorGo(2, CW, wheelSpeed);    // 2 - Right Back Wheel
   stopOneMotor(3);               // 3 - Left Back Wheel
}

void stopMoving() {
  // Initialize braked

  Serial.print("inches0: ");
  Serial.print(inches0);
  Serial.print("\n");
  
  Serial.print("inches1: ");
  Serial.print(inches1);
  Serial.print("\n");
  
  Serial.print("inches2: ");
  Serial.print(inches2);
  Serial.print("\n");  
  
  Serial.print("data in: ");
  Serial.print(dataIn);
  Serial.print("\n");
  
  dataOut = 0;
  Bluetooth2.write(dataOut);
  Bluetooth1.listen();
  Serial.print("data out: ");
  Serial.print(dataOut);
  Serial.print("\n");

  for (int i=0; i<4; i++) {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
    analogWrite(PWMpin[i], 0);
  }

  digitalWrite(actuatorUp,0);
  digitalWrite(actuatorDown,0);
}

void stopOneMotor(int i) {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
    analogWrite(PWMpin[i], 0);
}

/* motorGo() will set a motor going in a specific direction the motor will
 * continue going in that direction, at that speed until told to do otherwise.
 
 motor: this should be either 0 or 1, will selet which of the two motors to be controlled
 
 direct: Should be between 0 and 3, with the following result
       0: Brake to VCC
       1: Clockwise
       2: CounterClockwise
       3: Brake to GND
 
 pwm: should be a value between ? and 255, higher the number, the faster it'll go
 */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm) {
  if (motor <= 3) {
    if (direct <= 4) {       
      // Set inA[motor]
      if (direct <= 1) digitalWrite(inApin[motor], HIGH);
      else digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct == 0)||(direct == 2)) digitalWrite(inBpin[motor], HIGH);
      else digitalWrite(inBpin[motor], LOW);

      analogWrite(PWMpin[motor], pwm);
    }
  }
}

// ACTUATOR MOVEMENT
void goUp() {
  digitalWrite(actuatorDown,0);
  digitalWrite(actuatorUp,1);
}
  
void goDown() {
  digitalWrite(actuatorUp,0);
  digitalWrite(actuatorDown,1);
}



// PAINTING MODE
void paintLeftUp() {
  Serial.print("color: " + color);
  if (color == "cyan") dataOut = 1;
  else if (color == "magenta") dataOut = 2;
  else if (color == "yellow") dataOut = 3;
  else if (color == "black") dataOut = 4;
  Serial.print(" | dataOut: ");
  Serial.print(dataOut);
  Serial.print("\n");
  
  Bluetooth2.write(index);
  Bluetooth2.write(dataOut);
  Bluetooth1.listen();
    
  goUp();
  moveBackward();
}

void paintUp() {
  Serial.print("color: " + color);
  if (color == "cyan") dataOut = 1;
  else if (color == "magenta") dataOut = 2;
  else if (color == "yellow") dataOut = 3;
  else if (color == "black") dataOut = 4;
  Serial.print(" | dataOut: ");
  Serial.print(dataOut);
  Serial.print("\n");
 
  Bluetooth2.write(index);
  Bluetooth2.write(dataOut);
  Bluetooth1.listen();
    
  goUp();
}

void paintRightUp() {
  Serial.print("color: " + color);
  if (color == "cyan") dataOut = 1;
  else if (color == "magenta") dataOut = 2;
  else if (color == "yellow") dataOut = 3;
  else if (color == "black") dataOut = 4;
  Serial.print(" | dataOut: ");
  Serial.print(dataOut);
  Serial.print("\n");
 
  Bluetooth2.write(index);
  Bluetooth2.write(dataOut);
  Bluetooth1.listen();
    
  goUp();
  moveForward(); 
}

void paintLeft() {
  Serial.print("color: " + color);
  if (color == "cyan") dataOut = 1;
  else if (color == "magenta") dataOut = 2;
  else if (color == "yellow") dataOut = 3;
  else if (color == "black") dataOut = 4;
  Serial.print(" | dataOut: ");
  Serial.print(dataOut);
  Serial.print("\n");

  Bluetooth2.write(index);
  Bluetooth2.write(dataOut);
  Bluetooth1.listen();

  moveBackward();
}

void paintRight() {  
  Serial.print("color: " + color);
  if (color == "cyan") dataOut = 1;
  else if (color == "magenta") dataOut = 2;
  else if (color == "yellow") dataOut = 3;
  else if (color == "black") dataOut = 4;
  Serial.print(" | dataOut: ");
  Serial.print(dataOut);
  Serial.print("\n");
 
  Bluetooth2.write(index);
  Bluetooth2.write(dataOut);
  Bluetooth1.listen();
    
  moveForward();
}

void paintLeftDown() {
  Serial.print("color: " + color);
  if (color == "cyan") dataOut = 1;
  else if (color == "magenta") dataOut = 2;
  else if (color == "yellow") dataOut = 3;
  else if (color == "black") dataOut = 4;
  Serial.print(" | dataOut: ");
  Serial.print(dataOut);
  Serial.print("\n");
 
  Bluetooth2.write(index);
  Bluetooth2.write(dataOut);
  Bluetooth1.listen();
    
  goDown();
  moveBackward();
}

void paintDown() {
    Serial.print("color: " + color);
  if (color == "cyan") dataOut = 1;
  else if (color == "magenta") dataOut = 2;
  else if (color == "yellow") dataOut = 3;
  else if (color == "black") dataOut = 4;
  Serial.print(" | dataOut: ");
  Serial.print(dataOut);
  Serial.print("\n");
  
  Bluetooth2.write(index);
  Bluetooth2.write(dataOut);
  Bluetooth1.listen();
    
  goDown();
}

void paintRightDown() {
  Serial.print("color: " + color + "\n");
  if (color == "cyan") dataOut = 1;
  else if (color == "magenta") dataOut = 2;
  else if (color == "yellow") dataOut = 3;
  else if (color == "black") dataOut = 4;

  Serial.print("dataOut: ");
  Serial.print(dataOut);
  Serial.print("\n");

  Bluetooth2.write(index);
  Bluetooth2.write(dataOut);
  Bluetooth1.listen();
  
  goDown();
  moveForward();
}

long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}
