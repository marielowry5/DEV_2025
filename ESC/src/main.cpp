#include <Arduino.h>
#include <HardwareSerial.h>

// #define HWSERIAL Serial1  // serial port for debugging, bluetooth 

#define THROTTLE_PIN 14       // Throttle pin
#define THROTTLE_LOW 150      // These LOW and HIGH values are used to scale the ADC reading. More on this below
#define THROTTLE_HIGH 710
#define HALL_1_PIN 24       // hall A
#define HALL_2_PIN 25       // hall B
#define HALL_3_PIN 26       // hall C

// Pins from the Teensy to the gate drivers. AH = A high, etc
#define AH_PIN 15             // on board, pin 16, need to switch bc of pwm error
#define AL_PIN 17
#define BH_PIN 18
#define BL_PIN 19
#define CH_PIN 22             // on board, pin 20, need to switch bc of pwm error
#define CL_PIN 21

#define LED_PIN 13            // The teensy has a built-in LED on pin 13

#define HALL_OVERSAMPLE 4     // Hall oversampling count. More on this in the getHalls() function

uint8_t hallToMotor[8] = {255, 0, 2, 1, 4, 5, 3, 255}; // PROBLEM: how to know phase order? 

// uint8_t hallToMotor[8] = {255, 1, 3, 2, 5, 0, 4, 255};

/*

010
100


*/


//Zane edit
/*
From looking at the code, if I am understanding it correctly, since our sensors are 
always going the same direction and the positions don't change, I think we want to
comment out the 

"identifyHalls();"

at the end of setup. This is because it:

1. Introduces a 120 degree prediction of where it will be 2 states from now
 - that is great for high speeds, 
 - but for slow speeds it will mean the motor is always doing the wrong thing

2. We have a set hall effect orientation. Once we deterrmine the right hallToMotor
 - the wheels always turn the same way, meaning our order sould be constant

Instead, we want to out hallToMotor for our purposes. 
Below, I wrote two hallToMotor[] declarations. I'm not super confident that it will work, 
but if I am understanding the idea correctly one of the two hallToMotor arrays should
be correct. 



hall equals:
LSB - HallA Sensor
2nd LSB - HallB Sensor
3rd LSB - HallC Sensor

Lets say Halla starts with only halla on and it goes towards state hallb

Indexes 0 and 7 empty

State 0: 0 0 1 : Index 1    
State 1: 0 1 1 : Index 3
State 2: 0 1 0 : Index 2
State 3: 1 1 0 : Index 6
State 4: 1 0 0 : Index 4
State 5: 1 0 1 : Index 5

uint8_t hallToMotor[8] = {255, 0, 2, 1, 4, 5, 3, 255};

Could also go backwards/ towards hall 3

State 0: 0 0 1 : Index 1
State 1: 1 0 1 : Index 5
State 2: 1 0 0 : Index 4
State 3: 1 1 0 : Index 6
State 4: 0 1 0 : Index 2
State 5: 0 1 1 : Index 3

uint8_t hallToMotor[8] = {255, 0, 4, 5, 2, 1, 3, 255};

One of these should spin the motor forward, the other backward
*/

/*
Marie revision to motor state order 

Motor state order based on writePWM function 
Index 1: B high, A low 
Index 2: C high, A low 
Index 3: C high, B low
Index 4: A high, B low
Index 5: A high, C low
Index 6: B high, C low

State Index 1: B high, A low 
Index 2: C high, A low 
Index 3: C high, B low
Index 4: A high, B low
Index 5: A high, C low
Index 6: B high, C low

Need 
*/


// Forward declarations
void identifyHalls();
void writePWM(uint8_t motorState, uint8_t dutyCycle);
void writePhases(uint8_t ah, uint8_t bh, uint8_t ch, uint8_t al, uint8_t bl, uint8_t cl);
uint8_t getHalls();
uint8_t readThrottle();

void setup() {                // The setup function is called ONCE on boot-up
  Serial.begin(115200);
  Serial.println("test");


  analogReadResolution(10);  // Set ADC to 10-bit mode (0-1023)
  // HWSERIAL.begin(115200);
  

  pinMode(LED_PIN, OUTPUT);
  digitalWriteFast(LED_PIN, HIGH);

  pinMode(AH_PIN, OUTPUT);    // Set all PWM pins as output
  pinMode(AL_PIN, OUTPUT);
  pinMode(BH_PIN, OUTPUT);
  pinMode(BL_PIN, OUTPUT);
  pinMode(CH_PIN, OUTPUT);
  pinMode(CL_PIN, OUTPUT);

  analogWriteFrequency(AH_PIN, 8000); // Set the PWM frequency. Since all pins are on the same timer, this sets PWM freq for all
  analogWriteFrequency(BH_PIN, 8000); // Set the PWM frequency. Since all pins are on the same timer, this sets PWM freq for all
  analogWriteFrequency(CH_PIN, 8000); // Set the PWM frequency. Since all pins are on the same timer, this sets PWM freq for all


  pinMode(HALL_1_PIN, INPUT);         // Set the hall pins as input
  pinMode(HALL_2_PIN, INPUT);
  pinMode(HALL_3_PIN, INPUT);

  pinMode(THROTTLE_PIN, INPUT);
  
  //identifyHalls();                  // Uncomment this if you want the controller to auto-identify the hall states at startup!
}

void loop() {                         // The loop function is called repeatedly, once setup() is done
  
  uint8_t throttle = readThrottle();  // readThrottle() is slow. So do the more important things 200 times more often
  for(uint8_t i = 0; i < 200; i++)
  {  
    uint8_t hall = getHalls();              // Read from the hall sensors
    uint8_t motorState = hallToMotor[hall]; // Convert from hall values (from 1 to 6) to motor state values (from 0 to 5) in the correct order. This line is magic
    writePWM(motorState, 100);         // Actually command the transistors to switch into specified sequence and PWM value
  }

  // digitalWrite(LED_PIN, LOW);
  // delay(1000);
  // digitalWrite(LED_PIN, HIGH);
  // delay(1000);
}

/* Magic function to do hall auto-identification. Moves the motor to all 6 states, then reads the hall values from each one
 * 
 * Note, that in order to get a clean hall reading, we actually need to commutate to half-states. So instead of going to state 3, for example
 * we commutate to state 3.5, by rapidly switching between states 3 and 4. After waiting for a while (half a second), we read the hall value.
 * Finally, print it
 */

void identifyHalls()
{
  for(uint8_t i = 0; i < 6; i++)
  {
    uint8_t nextState = (i + 1) % 6;        // Calculate what the next state should be. This is for switching into half-states
    Serial.print("Going to ");
    Serial.println(i);
    // HWSERIAL.print("Going to ");
    // HWSERIAL.println(i);
    for(uint16_t j = 0; j < 200; j++)       // For a while, repeatedly switch between states
    { // TODO: try higher value for this 
      delay(1);
      writePWM(i, 20);
      delay(1);
      writePWM(nextState, 20);
    }
    // TODO: test this function when adding 1 instead of 2
    hallToMotor[getHalls()] = (i + 2) % 6;  // Store the hall state - motor state correlation. Notice that +2 indicates 90 degrees ahead, as we're at half states
  }
  
  writePWM(0, 0);                           // Turn phases off
  
  for(uint8_t i = 0; i < 8; i++)            // Print out the array
  {
    Serial.print(hallToMotor[i]);
    Serial.print(", ");
  }
  Serial.println();
}

/* This function takes a motorState (from 0 to 5) as an input, and decides which transistors to turn on
 * dutyCycle is from 0-255, and sets the PWM value.
 * 
 * Note if dutyCycle is zero, or if there's an invalid motorState, then it turns all transistors off
 */

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

/* Helper function to actually write values to transistors. For the low sides, takes a 0 or 1 for on/off
 * For high sides, takes 0-255 for PWM value
 */

void writePhases(uint8_t ah, uint8_t bh, uint8_t ch, uint8_t al, uint8_t bl, uint8_t cl) //THIS IS THE PROBLEM
{
  analogWrite(AH_PIN, ah);
  analogWrite(BH_PIN, bh);
  analogWrite(CH_PIN, ch);
  digitalWriteFast(AL_PIN, al);
  digitalWriteFast(BL_PIN, bl);
  digitalWriteFast(CL_PIN, cl);
}

/* Read hall sensors WITH oversamping. This is required, as the hall sensor readings are often noisy.
 * This function reads the sensors multiple times (defined by HALL_OVERSAMPLE) and only sets the output
 * to a 1 if a majority of the readings are 1. This really helps reject noise. If the motor starts "cogging" or "skipping"
 * at low speed and high torque, try increasing the HALL_OVERSAMPLE value
 * 
 * Outputs a number, with the last 3 binary digits corresponding to hall readings. Thus 0 to 7, or 1 to 6 in normal operation
 */

uint8_t getHalls()
{
  uint8_t hallCounts[] = {0, 0, 0};
  for(uint8_t i = 0; i < HALL_OVERSAMPLE; i++) // Read all the hall pins repeatedly, tally results 
  {
    hallCounts[0] += digitalReadFast(HALL_1_PIN);
    hallCounts[1] += digitalReadFast(HALL_2_PIN);
    hallCounts[2] += digitalReadFast(HALL_3_PIN);
  }

  uint8_t hall = 0;
  
  if (hallCounts[0] >= HALL_OVERSAMPLE / 2)     // If votes >= threshold, call that a 1
    hall |= (1<<0);                             // Store a 1 in the 0th bit
  if (hallCounts[1] >= HALL_OVERSAMPLE / 2)
    hall |= (1<<1);                             // Store a 1 in the 1st bit
  if (hallCounts[2] >= HALL_OVERSAMPLE / 2)
    hall |= (1<<2);                             // Store a 1 in the 2nd bit

  return hall & 0x7;                            // Just to make sure we didn't do anything stupid, set the maximum output value to 7
}

/* Read the throttle value from the ADC. Because our ADC can read from 0v-3.3v, but the throttle doesn't output over this whole range,
 * scale the throttle reading to take up the full range of 0-255
 */

uint8_t readThrottle()
{
  int32_t adc = analogRead(THROTTLE_PIN); // Note, analogRead can be slow!
  adc = (adc - THROTTLE_LOW) << 8;
  adc = adc / (THROTTLE_HIGH - THROTTLE_LOW);

  adc /= 395.0/255;

  // Serial.println(adc);

  if (adc > 255) // Bound the output between 0 and 255
    return 255;

  if (adc < 0)
    return 0;
  
  return adc;
}