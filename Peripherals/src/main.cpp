#include <Arduino.h>

#define FL 0
#define FR 1
#define FLH 2
#define FRH 3
#define BLH 4
#define BRH 5
#define fan 6
#define horn 7
#define BR 8
#define runningBR 9
#define BL 10
#define runningBL 11

#define sBrake A0
#define sFan A1
#define sTurnLeft A2
#define sTurnRight A3
#define sHazard A4
#define sHL A5
#define sRunning A6
#define sHorn A7
int leftPriority = 0;
int rightPriority = 0;
int hazardPriority = 0;

void setup() {
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
      digitalWrite(FLH, HIGH); //front left hazard
      digitalWrite(FRH, HIGH); //front right hazard
      digitalWrite(BLH, HIGH); //back left hazard
      digitalWrite(BRH, HIGH); //back right hazard
      delay(300); //BLINKING
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
