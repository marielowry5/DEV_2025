//-----------------------------------------------------------------
//Libraries

#include <Arduino.h>  //Basic Arduino Library
#include <SPI.h>      //SPI Communication Protocal 

//-----------------------------------------------------------------
//Registers

//Gyro Registers
#define GYRO_CONFIG 0x1B //Gyro config address
#define GYRO_ZOUT_H 0x47 //Yaw Axis | High Byte

//Accelerometer Registers
#define ACCEL_CONFIG 0x1C //Accelerometer config address
#define ACCEL_XOUT_H 0x3B //Accerleration in the turning direction | High Byte
#define ACCEL_YOUT_H 0x3D //Acceleration in forward and back tilt  | High Byte

//Power Registers
#define PWR_MGMT_1 0x6B   //Used to wake up sensor


//-----------------------------------------------------------------
//Pin Declarations
#define CS 10   //chip select
#define MOSI 11 //board to IMU
#define MISO 12 //IMU to Board
#define SCK 13  //SPI Clock

#define throttleIn A0 //throttle in from pedal (41) (0-5)
#define RXR 28 //Read Velocity from Right Wheel in uint16 in inches/sec (Serial 7)
#define RXL 34 //Read Velocity from Right Wheel in uint16 in inches/sec (Serial 8)
#define throttleRight 29 //AnalogVoltage out to Right Motor
#define throttleLeft 33 //AnalogVoltage out to Left Motor


//--------------------Global Variables---------------------------------------------
//Car Constants
const int wheelBase= 100; //from front wheel axle to back wheel axle (inches)
const int trackWidth = 50; //horizontal distance between wheels (inches)

//Control Constants
const int adjustSpeed = 30; //sets the analogWrite to decrease by 30 intervals by differential

//Gyro Constants
unsigned long lastGyroTime = 0; //stores last time gyro was updated to get dt
const float gyroSensitivity = 131;    //Gyrosensitivy scale factor (LSB/(º/s)) (Register Map 3.1) | Means that for every º/s, the least significant bit changes by 131
const float accelSensitivity = 16384.0;   //Acceleromter Sensitity Factor (LSB/g) (Register Map 3.2) | Means that for every g multiple of gravity, the least significant bit changes by 16384

//Speed Variables
int actualRightSpeed=0;
int actualLeftSpeed=0;

//Kalman Fiilter Variables
float estimatedYawAngle = 0;    //estimated Yaw Angle after filtering
float bias = 0;                 //Total estimated bias of the gyroscope (the gyroscope might consistently have a bias of +5 degrees / second, this bias gets closer to the bias of the sensor)

float ecm[2][2]={{1,0},{0,1}}; //Error Covariance Matrix tracks the error/confidence in the measurements. Initialized with no trust in estimates
//[0,0]: 0-1 the uncertainty in the gyroscope estimate. 0 Means no uncertainty, 1 means more uncertainty thus trust measurement more. 
//[1,1] 0 to 1 how much we trust the bias estimate (0 full trust) vesus uncertaintly of bias estimate (1 full uncertainty)
//[0,1] and [1,0]: Link the error in the angle to errors in the bias. + is errors in the bias mean high positive errors in bias, - is errors in the angle mean high negative error in bias.

float gyroNoise= 0.1*0.1; //Noise of 0.1(º/s-rms) (Sheet 3.1) (standard deviation). Squaring gets the variance.
float gyroBias= 0.003; //Cooresponds to how fast the bias is expected to change. A low value means bias is slower to respond to changes in bias. A higher value is used of the bias is expected to be slow changing. 0.003 is a low value.
float accelNoise = 0.03; //Noise of 8 (mg-rms) (Sheet 3.2) (standard deviation). Squaring gets variance

int differential[2]={0,0};


//-----------------------------------------------------------------
//Function Declarations
void writeRegister(uint8_t address, uint8_t data);
uint8_t readRegister(uint8_t address);
int16_t readAccelYaw();
int16_t readGyroVelocity();
void kalmanFilter(float accelYaw, float gyroVelocity, float dt);
void differentialFunction(float angle);
uint16_t readSpeedsUART();

//-----------------------------------------------------------------
//Differential


void differentialFunction(float angle){
  int centerRadius=100000; //set default to a mile
  if(abs(angle)>2){        //a little tolerance for steering wheel angle
    centerRadius = (int) wheelBase/(tan(angle * (PI / 180.0))); //the turning radius of the center of the car
  }

  int leftRadius = centerRadius - trackWidth/2; //turning radius of back left wheel
  int rightRadius = centerRadius + trackWidth/2; //turning radius of back right wheel

  while(Serial.available()>0){
    int recieved= Serial.parseInt();
    if ((int) recieved <= (int)1000){
      actualLeftSpeed= recieved+0;
    } else{
      actualRightSpeed=recieved-1000;
    }
    while(Serial.available()>0){
      Serial.read();
    }
  }

  int actualCenterSpeed=(actualLeftSpeed+actualRightSpeed)/2; //get the center of car speed
  int goalLeftSpeed = actualCenterSpeed * (leftRadius/ (float) centerRadius);
  int goalRightSpeed = actualCenterSpeed * (rightRadius/ (float) centerRadius);

  if(goalLeftSpeed!=actualLeftSpeed)      differential[0]+=(goalLeftSpeed-actualLeftSpeed) / abs(goalLeftSpeed-actualLeftSpeed); //decrease ofset of left if too fast
  if(goalRightSpeed!=actualRightSpeed)    differential[1]+=(goalRightSpeed-actualRightSpeed) / abs(goalRightSpeed-actualRightSpeed); //decrease ofset of right if too fast
  if(differential[0]>0)differential[0]=0;  //prevent speeding up the left wheel
  if(differential[1]>0)differential[1]=0;  //prevent speeding up the right wheel

  int voltageIn = (int) (analogRead(throttleIn)); //Reads throttle in from 0-2.5V and converts to 1-1023
  //Serial.println(voltageIn);
  if (voltageIn>900) voltageIn=1023; //ensures max is 1023 and allows max speed to hold
  if (voltageIn<100) voltageIn = 0;  //ensures min is 0 and allows off to hold

  analogWrite(throttleLeft,voltageIn+differential[0]*adjustSpeed);
  analogWrite(throttleRight,voltageIn+differential[1]*adjustSpeed);

  Serial.print("Left Speed: ");
  Serial.print(actualLeftSpeed);
  Serial.print(" Right Speed: ");
  Serial.print(actualRightSpeed);  
  Serial.print(" | Center Speed: ");
  Serial.print(actualCenterSpeed);
  Serial.print(" | Angle: ");
  Serial.print(angle);
  Serial.print(" | Turn Radius: ");
  Serial.print(centerRadius);
  Serial.print(" | Goal Left Speed: ");
  Serial.print(goalLeftSpeed);
  Serial.print(" | Goal Right Speed: ");
  Serial.print(goalRightSpeed);
  Serial.print(" | Left Differential: ");
  Serial.print(differential[0]);
  Serial.print(" | Right Differential: ");
  Serial.println(differential[1]);
}

//-----------------------------------------------------------------
//Kalman Filter
void kalmanFilter(float accelYaw, float gyroVelocity, float dt){
  estimatedYawAngle+=(gyroVelocity-bias)*dt; //takes the gyroVelocity, subtracts the sensor's bias in that direction, and integrates over time tog et estimated angle
  
  ecm[0][0]+= (gyroNoise*dt)+(ecm[1][1]*dt*dt)-(ecm[0][1]*dt)-(ecm[1][0]*dt); //changes the uncertainty in the gyro estimate
  //adds a fixed rate based off of noise, adds a estimate based on a quadratic uncertainty in the bias, subtracts the strength of relationships between the angle error bias error

  ecm[1][0]-=dt*ecm[1][1]; //changes the correlation between the angle and bias. Decrease correlation by the uncertainty in the bias.
  ecm[0][1]=dt*ecm[1][1]; // keeps matrix symmetrcial

  ecm[1][1]+= gyroBias*dt; //adjusts the bias by how much bias can change integrated over time

  float totalEstimatedError = ecm[0][0]+accelNoise; //adds gyroscope estimate error with accelerometer error
  float kalmanGains[2];
  kalmanGains[0]= (ecm[0][0]/totalEstimatedError); // (uncertainty of gyro estimate / over total uncertainty) 
  kalmanGains[1]= (ecm[1][0]/totalEstimatedError);// higher the correlation between angle in error and bias, the higher the bias gain is adjusted

  float difference=accelYaw-estimatedYawAngle; //difference between measured angle and estimated angle
  estimatedYawAngle += kalmanGains[0]*difference; //move the estimate closer to the measured based on how uncertain the estimte is. (higher kalmanGain, more towards estimate)
  bias += kalmanGains[1]*difference; //how much the bias is changed to explain the gap in the difference.

  ecm[0][0]-=kalmanGains[0]*ecm[0][0]; //changes the angle uncertainty by a factor of the (gyro uncertainty / total uncertainty)
  ecm[0][1]-=kalmanGains[0]*ecm[0][1]; //changes the correlation between angle change and bias change to be based off of how accurate the estimate is
  ecm[1][0]=kalmanGains[1]*ecm[0][0]; //keeps matrix symmetrical
  ecm[1][1]-=kalmanGains[1]*ecm[0][1]; //changes the bias uncertainty by a factor of the (bias uncertainty / total uncertainty)

}

//-----------------------------------------------------------------
//read from register of IMU
int16_t readGyroVelocity(){
  return (readRegister(GYRO_ZOUT_H)<<8 | readRegister(GYRO_ZOUT_H+1)); //read the angular acceleration of the gyroscope.
}

//-----------------------------------------------------------------
//read from register of IMU
int16_t readAccelYaw(){
  int16_t accelXRaw = ((readRegister(ACCEL_XOUT_H)<<8) | (readRegister(ACCEL_XOUT_H+1)));  //read the 16 bit binary X acceleration value
  int16_t accelYRaw = ((readRegister(ACCEL_YOUT_H)<<8) | (readRegister(ACCEL_YOUT_H+1)));  //read the 16 bit binary Y acceleration value
  
  float accelXConverted = accelXRaw/accelSensitivity; //divides the number by sensitivity constant to get X acceleration is multiple of gravity
  float accelYConverted = accelYRaw/accelSensitivity; //divides the number by sensitivity constant to get Y acceleration is multiple of gravity

  return atan2(accelYConverted, accelXConverted) * (180 / M_PI); //angle calculated with opposite/adjacent (y/x) and the converted from radians to degrees
}


//-----------------------------------------------------------------
//write to register of IMU
void writeRegister(uint8_t address, uint8_t data){
  digitalWrite(CS, LOW);  //tell IMU we are communicating with it
  SPI.transfer(address);  //send the address we hope to send data to
  SPI.transfer(data);     //send data to register
  digitalWrite(CS, HIGH); //stop communicating
}


//-----------------------------------------------------------------
//read from register of IMU
uint8_t readRegister(uint8_t address){
  digitalWrite(CS,LOW);               //tell IMU we are communicating with it
  SPI.transfer(address | 0x80);       //send the address we hope to read from. The 0x80 sets the most significant digit to one to ensure read operation.
  uint8_t data = SPI.transfer(0x00);  //sends a filler bit to fill space while getting data
  digitalWrite(CS, HIGH);             //stop communicating
  return data;                        //return value
}

//-----------------------------------------------------------------
//Setup Function
void setup() {

  Serial.begin(115200);   //initialize serial communication
  SPI.begin();            //initialize SPI interface

  //Serial7.begin(115200);     //initialize leftMotorRead
  //erial8.begin(115200);     //initialize rightMotorRead

  pinMode(CS, OUTPUT);    //Set Chip Select to Output
  digitalWrite(CS, HIGH); //turn off SPI communication by default

  writeRegister(PWR_MGMT_1, 0x00); //Wake Up IMU
  writeRegister(GYRO_CONFIG, 0x00); //sets GYRO to ±250°/s
  //writeRegister(ACCEL_CONFIG, 0x00); //sets ACCEL to 2g

  lastGyroTime=millis(); //initialize time since last update at t=0
}

//-----------------------------------------------------------------
//Loop Function
void loop() {

  //---------------------Gets Angle of Steering Wheel------------------------------//
  int16_t gyroRaw = readGyroVelocity(); //get binary gyro value
  float gyroDPS = gyroRaw / gyroSensitivity;  //convert to degrees per second

  int16_t accelYaw = readAccelYaw(); //get accerometer degreeth

  unsigned long t=millis(); //get time
  float dt=(t-lastGyroTime)/1000.0; //get change in time in seconds
  lastGyroTime=t;

  kalmanFilter(accelYaw, gyroDPS,dt); //run kalman fiter
  //Serial.println(estimatedYawAngle); //print angle for debugging

  //TO DOOOO
  float ccwAngle = -1.0 *estimatedYawAngle; // TO DO: CHANGE TO -1 if yaw angle not CCW Positive
  //TO DO: Input formula to convert steering wheel angle to actual angle

  //int voltageIn = analogRead(throttleIn);
  //Serial.println(voltageIn);

  if(Serial.available()>=1) differentialFunction(ccwAngle);
  delay(1);
}