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

//-----------------------------------------------------------------
//Global Variables
unsigned long lastGyroTime = 0; //stores last time gyro was updated to get dt
float yawAngle = 0;             //stores steering wheel angle
float gyroSensitivity = 131;    //Gyrosensitivy scale factor (LSB/(º/s)) (Register Map 3.1) | Means that for every º/s, the least significant bit changes by 131
float accelSensisity = 16384;   //Acceleromter Sensitity Factor (LSB/g) (Register Map 3.2) | Means that for every g multiple of gravity, the least significant bit changes by 16384

//-----------------------------------------------------------------
//Function Declarations
void writeRegister(uint8_t address, uint8_t data);
uint8_t readRegister(uint8_t address);


void setup() {

  Serial.begin(115200);   //initialize serial communication
  SPI.begin();            //initialize SPI interface

  pinMode(CS, OUTPUT);    //Set Chip Select to Output
  digitalWrite(CS, HIGH); //turn off SPI communication by default

  writeRegister(PWR_MGMT_1, 0x00); //Wake Up IMU
  writeRegister(GYRO_CONFIG, 0x00); //sets GYRO to ±250°/s
  writeRegister(ACCEL_CONFIG, 0x00); //sets ACCEL to 2g

}

void loop() {
  // put your main code here, to run repeatedly:
}

//-----------------------------------------------------------------
//read from register of IMU
uint16_t readGyroYaw(){
  return (readRegister(GYRO_ZOUT_H)<<8 | readRegister(GYRO_ZOUT_H+1)); //read the angular acceleration of the gyroscope.
}

//-----------------------------------------------------------------
//read from register of IMU
uint16_t readAccelYaw(){
  uint16_t accelXBinary = ((readRegister(ACCEL_XOUT_H)<<8) | (readRegister(ACCEL_XOUT_H+1)));  //read the 16 bit binary X acceleration value
  uint16_t accelYBinary = ((readRegister(ACCEL_YOUT_H)<<8) | (readRegister(ACCEL_YOUT_H+1)));  //read the 16 bit binary Y acceleration value
  
  float accelXConverted = accelXBinary/accelSensisity; //divides the number by sensitivity constant to get X acceleration is multiple of gravity
  float accelYConverted = accelYBinary/accelSensisity; //divides the number by sensitivity constant to get Y acceleration is multiple of gravity

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

