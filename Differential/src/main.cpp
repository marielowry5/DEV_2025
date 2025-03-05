//-----------------------------------------------------------------
//Libraries

#include <Arduino.h>  //Basic Arduino Library
#include <SPI.h>      //SPI Communication Protocal 

//-----------------------------------------------------------------
//Registers

//Gyro Registers
#define GYRO_CONFIG 0x1B //Gyro config to change dps to 250
#define GYRO_ZOUT_H 0x47 //Yaw Axis | High Byte

//Accelerometer Registers
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
//Function Declarations
void writeRegister(uint8_t address, uint8_t data);
uint8_t readRegister(uint8_t address);



void setup() {

  Serial.begin(115200);   //initialize serial communication
  SPI.begin();            //initialize SPI interface

  pinMode(CS, OUTPUT);    //Set Chip Select to Output
  digitalWrite(CS, HIGH); //turn off SPI communication by default

  writeRegister(PWR_MGMT_1, 0x00); //Wake Up IMU
  writeRegister()
}

void loop() {
  // put your main code here, to run repeatedly:
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

