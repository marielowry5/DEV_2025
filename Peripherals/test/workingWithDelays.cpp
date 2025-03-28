#include <Arduino.h> //import Arduino library

//Defining Light, Fan & Horn variables
#define FL 0  // Front Left Headlight
#define FR 1  //Front Right Headlight
#define FLH 2 //Front Left Hazard
#define FRH 3 //Front Right Hazard
#define BLH 4 //Back Left Hazard
#define BRH 5 //Back Left Hazard
#define fan 6 //Fan
#define horn 7  //Horn
#define BR 8  //Right Braking Light
#define runningBR 9 // Right Running Back light
#define BL 10 // Left Brake Light
#define runningBL 11  // Left Running Back Light

//Defining Switches
#define sBrake A0 //Brake Switch
#define sFan A1 //Fan Switch
#define sTurnLeft A2  //Left Turn Switch
#define sTurnRight A3 //Right Turn Switch
#define sHazard A4  //Hazard Switch
#define sHL A5  //Headlight Switch
#define sRunning A6 //Running Switch
#define sHorn A7  //Horn Switch
int leftPriority = 0; //initial left turn signal switch value (0 means switch is not turned on and 1 means switch is turned on)
int rightPriority = 0;  //initial right turn signal switch value (0 means switch is not turned on and 1 means switch is turned on)
int hazardPriority = 0; //initial hazard signal switch value (0 means switch is not turned on and 1 means switch is turned on)

unsigned long myTime; //variable that will hold the time it is when the switch is turned on
unsigned long newTime = myTime + 500; //when newTime = myTime, it will stop whatever signal is turned on.

void setup() {
  // setting all the lights, fan, and horn to be output mode
  pinMode(FL, OUTPUT);
  pinMode(FR, OUTPUT);
  pinMode(FLH, OUTPUT);
  pinMode(FRH, OUTPUT);
  pinMode(BLH, OUTPUT);
  pinMode(BRH, OUTPUT);
  pinMode(BL, OUTPUT);
  pinMode(BR, OUTPUT);
  pinMode(fan, OUTPUT);
  pinMode(horn, OUTPUT);

  // setting signaling, fan, and brake switches to input pullup
  pinMode(sHazard, INPUT_PULLUP);
  pinMode(sTurnRight, INPUT_PULLUP);
  pinMode(sTurnLeft, INPUT_PULLUP);
  pinMode(sFan, INPUT_PULLUP);
  pinMode(sBrake, INPUT_PULLUP);
  Serial.begin(9600); //initialize serial monitor
}

void loop() {
  if(analogRead(sRunning) == 0){ //running lights switch
    Serial.print("RunLights: On | ");
    digitalWrite(runningBL, HIGH); //running brake left
    digitalWrite(runningBR, HIGH); //running brake right
  }

  else{ //if switch is off turn running lights off
    Serial.print("RunLights: Off | ");
    digitalWrite(runningBL, LOW);
    digitalWrite(runningBR, LOW);
  }

  if(analogRead(sHL) == 0){ //headlights switch
    Serial.print("HeadLights: On | ");
    digitalWrite(FL, HIGH); //front left headlight
    digitalWrite(FR, HIGH); //front right headlight
  }
  else{ //if switch is off turn headlights off
    Serial.print("HeadLights: Off | ");
    digitalWrite(FL, LOW);
    digitalWrite(FR, LOW);
  }

  if(digitalRead(sHazard) == LOW){ //all hazards switch
    Serial.print("Hazards: On | ");
    if(leftPriority == 0 && rightPriority == 0){ //if left and right switches aren't on
      hazardPriority = 1;
      myTime = millis();
      digitalWrite(FLH, HIGH); //front left hazard
      digitalWrite(FRH, HIGH); //front right hazard
      digitalWrite(BLH, HIGH); //back left hazard
      digitalWrite(BRH, HIGH); //back right hazard
      while(myTime+300 < newTime)
      {
        newTime--;
      }
      digitalWrite(FLH, LOW);
      digitalWrite(FRH, LOW);
      digitalWrite(BLH, LOW);
      digitalWrite(BRH, LOW);
      delay(300);
    }
  }
  else{ //if switch is off turn all hazards off
    Serial.print("Hazards: Off | ");
    hazardPriority = 0;
    digitalWrite(FLH, LOW);
    digitalWrite(FRH, LOW);
    digitalWrite(BLH, LOW);
    digitalWrite(BRH, LOW);
  }

  if(digitalRead(sTurnRight) == LOW){ //right turn switch
    Serial.print("RightTurn: On | ");
    if(leftPriority == 0 && hazardPriority == 0){ //if left turn and hazard switches are off
      rightPriority = 1;
      digitalWrite(FRH, HIGH); //front right hazard
      digitalWrite(BRH, HIGH); //back right hazard
      delay(300); //BLINKING
      digitalWrite(FRH, LOW);
      digitalWrite(BRH, LOW);
      delay(300);
    }
  }
  else{ //if right turn switch is off, turn off right turn lights
    Serial.print("RightTurn: Off | ");
    rightPriority = 0;
    digitalWrite(FRH, LOW);
    digitalWrite(BRH, LOW);
  }

  if(digitalRead(sTurnLeft) == LOW){ //left turn switch
    Serial.print("LeftTurn: On | ");
    if(rightPriority == 0 && hazardPriority == 0){ //if right turn and hazard switches are off
      leftPriority = 1;
      digitalWrite(FLH, HIGH); //front left hazard
      digitalWrite(BLH, HIGH); //back left hazard
      delay(300); //BLINKING
      digitalWrite(FLH, LOW);
      digitalWrite(BLH, LOW);
      delay(300);
    }
  }
  else{ //if left turn switch is off, turn off left turn lights
    Serial.print("LeftTurn: Off | ");
    leftPriority = 0;
    digitalWrite(FLH, LOW);
    digitalWrite(BLH, LOW);
  }

  if(digitalRead(sFan) == LOW){ //fan switch
    Serial.println("Fan: On | ");
    digitalWrite(fan, HIGH); //fan on
  }
  else{ //if fan switch is off, turn fan off
    Serial.print("Fan: Off | ");
    digitalWrite(fan, LOW);
  }

  if(digitalRead(sBrake) == LOW){ //brakes switch (from pedal sensor)
    Serial.print("Brake: On | ");
    digitalWrite(BL, HIGH); //left brake light
    digitalWrite(BR, HIGH); //right brake light
  }
  else{ //if brakes aren't pushed, turn brake lights off
    Serial.print("Brake: Off | ");
    digitalWrite(BL, LOW);
    digitalWrite(BR, LOW);
  }

  if(analogRead(sHorn) == 0){ //horn switch
    Serial.println("Horn: On | ");
    digitalWrite(horn, HIGH); //horn on
  }
  else{ //if horn switch is off, turn horn off
    Serial.println("Horn: Off | ");
    digitalWrite(horn, LOW);
  }
  delay(1000);
}
