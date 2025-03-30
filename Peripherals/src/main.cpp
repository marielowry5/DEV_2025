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
#define BL 10 // Left Brake Light

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

long dutySwitchTime=0;
bool dutyOn=true;

long hornTime=-1000;
long blinkTime=-500;
bool blinkOn=true;

bool runningLights=false;
bool headlights=false;
bool leftTurn=false;
bool rightTurn=false;
bool hazards=false;
bool windFan = false;
bool brake = false;
bool honk = false;

void updateStates(long currentTime);
void driveTransistors(long currentTime);

void setup() {
  //Serial.begin(9600);
  // setting all the lights, fan, and horn to be output mode
  pinMode(FL, OUTPUT); //headlights 
  pinMode(FR, OUTPUT); //running
  pinMode(FLH, OUTPUT); //hazard + left
  pinMode(FRH, OUTPUT); //hazard + right
  pinMode(BLH, OUTPUT); //hazard + left
  pinMode(BRH, OUTPUT); //hazard + right
  pinMode(BL, OUTPUT); //brake + running
  pinMode(BR, OUTPUT); //brake + running
  pinMode(fan, OUTPUT); //fan
  pinMode(horn, OUTPUT); //horn

  // setting signaling, fan, and brake switches to input pullup
  pinMode(sHL, INPUT_PULLUP);
  pinMode(sHazard, INPUT_PULLUP);
  pinMode(sTurnRight, INPUT_PULLUP);
  pinMode(sTurnLeft, INPUT_PULLUP);
  pinMode(sFan, INPUT_PULLUP);
  pinMode(sBrake, INPUT_PULLUP);
}

void loop() {
  //Serial.println("test");
  long currentTime=millis();
  updateStates(currentTime);

  if(currentTime>(1+dutySwitchTime)){ //if 5 ms passed, switch duty on to off
    dutyOn= !dutyOn; 
    dutySwitchTime=currentTime;
  }

  if(currentTime>(750+blinkTime)){
    blinkOn=!blinkOn;
    blinkTime=currentTime;
  }

  driveTransistors(currentTime);
}

void updateStates(long currentTime){
  if(analogRead(sRunning) == 0){ //running lights switch
    runningLights=true;
  }
  else{ //if switch is off turn running lights off
    runningLights=false;
  }

  if(digitalRead(sHL) == LOW){ //headlights switch
    headlights=true;
  }
  else{ //if switch is off turn headlights off
     headlights=false;
  }

  if(digitalRead(sHazard) == LOW){ //all hazards switch
    if(leftPriority == 0 && rightPriority == 0){ //if left and right switches aren't on
      hazardPriority = 1;
      hazards=true;
    }
  }
  else{ //if switch is off turn all hazards off
    hazardPriority = 0;
    hazards=false;
  }

  if(digitalRead(sTurnRight) == LOW){ //right turn switch
    if(leftPriority == 0 && hazardPriority == 0){ //if left turn and hazard switches are off
      rightPriority = 1;
      rightTurn=true;
    }
  }
  else{ //if right turn switch is off, turn off right turn lights
    rightPriority = 0;
    rightTurn=false;
  }

  if(digitalRead(sTurnLeft) == LOW){ //left turn switch
    if(rightPriority == 0 && hazardPriority == 0){ //if right turn and hazard switches are off
      leftPriority = 1;
      leftTurn=true;
    }
  }
  else{ //if left turn switch is off, turn off left turn lights
    leftPriority = 0;
    leftTurn=false;
  }

  if(digitalRead(sFan) == LOW){ //fan switch
    windFan=true;
  }
  else{ //if fan switch is off, turn fan off
    windFan=false;
  }

  if(digitalRead(sBrake) == LOW){ //brakes switch (from pedal sensor)
    brake=true;
  }
  else{ //if brakes aren't pushed, turn brake lights off
    brake=false;
  }

  if(analogRead(sHorn) <= 0){ //horn switch
    hornTime=currentTime;
  }
}


void driveTransistors(long currentTime){
  digitalWrite(fan, windFan); //turns 

  digitalWrite(BL, (dutyOn&runningLights) || brake); //dim if running, always on when braking
  digitalWrite(BR, (dutyOn&runningLights) || brake); //dim if running, always on when breaking

  digitalWrite(FR,headlights); //turn on headlights
  digitalWrite(FL,headlights); //turn on headlights

  if(currentTime<(hornTime+250)){ //horn for 1/2 second
    digitalWrite(horn, HIGH);
    //Serial.println("Horn");
  }else{
    digitalWrite(horn, LOW);
  }

  digitalWrite(FLH, blinkOn & (hazards || leftTurn)); //turn on front left blinker
  digitalWrite(FRH, blinkOn & (hazards || rightTurn)); //turn on back right blinker

  digitalWrite(BLH, blinkOn & (hazards || leftTurn)); //turn on back left blinker
  digitalWrite(BRH, blinkOn & (hazards || rightTurn)); //turn on back right blinker
}