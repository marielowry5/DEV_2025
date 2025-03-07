// Test script to identify hall sensor order and print output to serial monitor
// Run to determine order of hallToMotor[8] array in main 
// Ensure serial monitor functionality so this program is useful 
// pwm must be functional for this to work

#include <Arduino.h>

#define HALL_1_PIN 24       // hall A
#define HALL_2_PIN 25       // hall B
#define HALL_3_PIN 26       // hall C

#define LED_PIN 13            // The teensy has a built-in LED on pin 13

#define HALL_OVERSAMPLE 4     // Hall oversampling count. More on this in the getHalls() function

#define AH_PIN 22             // Pins from the Teensy to the gate drivers. AH = A high, etc
#define AL_PIN 23
#define BH_PIN 18
#define BL_PIN 19
#define CH_PIN 20
#define CL_PIN 21


uint8_t hallToMotor[8] = {255, 255, 255, 255, 255, 255, 255, 255}; 


void setup() {
    Serial.begin(115200); // set baud rate for serial monitor
    analogReadResolution(10);  // Set ADC to 10-bit mode (0-1023), makes teensy 3.2 code compatible with 4.1 code

    pinMode(LED_PIN, OUTPUT);
    digitalWriteFast(LED_PIN, HIGH); // set on-board LED to high when program running

    pinMode(HALL_1_PIN, INPUT);         // Set the hall pins as input
    pinMode(HALL_2_PIN, INPUT);
    pinMode(HALL_3_PIN, INPUT);
  
    pinMode(AH_PIN, OUTPUT);    // Set all PWM pins as output
    pinMode(AL_PIN, OUTPUT);
    pinMode(BH_PIN, OUTPUT);
    pinMode(BL_PIN, OUTPUT);
    pinMode(CH_PIN, OUTPUT);
    pinMode(CL_PIN, OUTPUT);
  
    analogWriteFrequency(AH_PIN, 8000); // Set the PWM frequency
    analogWriteFrequency(BH_PIN, 8000); // Set the PWM frequency
    analogWriteFrequency(CH_PIN, 8000); // Set the PWM frequency
  

    identifyHalls()         // call hall identification section, this will print hallToMotor[] array


}

void loop() {

}

void identifyHalls()
{
  for(uint8_t i = 0; i < 6; i++)
  {
    uint8_t nextState = (i + 1) % 6;        // Calculate what the next state should be. This is for switching into half-states
    Serial.print("Going to ");
    Serial.println(i);

    for(uint16_t j = 0; j < 200; j++)       // For a while, repeatedly switch between states
    {
      delay(1);
      writePWM(i, 20); // call writePWM function
      delay(1);
      writePWM(nextState, 20);
    }
    hallToMotor[getHalls()] = (i + 2) % 6;  // Store the hall state - motor state correlation. Notice that +2 indicates 90 degrees ahead, as we're at half states
    // based on motor is after switching through all states, hall sensors will be in correct next state? 
}
  
  writePWM(0, 0);                           // Turn phases off
  
  for(uint8_t i = 0; i < 8; i++)            // Print out the array
  {
    Serial.print(hallToMotor[i]);
    Serial.print(", ");
  }
  Serial.println();
}


void writePWM(uint8_t motorState, uint8_t dutyCycle)
{
  if(dutyCycle == 0)                          // If zero throttle, turn all off
    motorState = 255;

  if(motorState == 0)                         // LOW A, HIGH B
      writePhases(0, dutyCycle, 0, 1, 0, 0);
  else if(motorState == 1)                    // LOW A, HIGH C
      writePhases(0, 0, dutyCycle, 1, 0, 0);
  else if(motorState == 2)                    // LOW B, HIGH C
      writePhases(0, 0, dutyCycle, 0, 1, 0);
  else if(motorState == 3)                    // LOW B, HIGH A
      writePhases(dutyCycle, 0, 0, 0, 1, 0);
  else if(motorState == 4)                    // LOW C, HIGH A
      writePhases(dutyCycle, 0, 0, 0, 0, 1);
  else if(motorState == 5)                    // LOW C, HIGH B
      writePhases(0, dutyCycle, 0, 0, 0, 1);
  else                                        // All off
      writePhases(0, 0, 0, 0, 0, 0);
}

void writePhases(uint8_t ah, uint8_t bh, uint8_t ch, uint8_t al, uint8_t bl, uint8_t cl) 
{
  analogWrite(AH_PIN, ah);
  analogWrite(BH_PIN, bh);
  analogWrite(CH_PIN, ch);
  digitalWriteFast(AL_PIN, al);
  digitalWriteFast(BL_PIN, bl);
  digitalWriteFast(CL_PIN, cl);
}