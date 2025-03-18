// function to test pwm pin output of teensy 4.1
// loops through each pwm pin used in main and outputs square wave at 50% duty cycle
// should isolate motor controller failure to software issue

#include <Arduino.h>

const int pwmPins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 18, 19, 22, 23, 24, 25, 28, 29, 33, 36, 37};
const int numPins = sizeof(pwmPins) / sizeof(pwmPins[0]); // size of array in bytes divided by size of one array element, gives array length
const int pwmFrequency = 1000; // Set PWM frequency to 1 kHz
const int dutyCycle = 128; // 50% duty cycle (0-255 for 8-bit resolution)

void setup() {
    for (int i = 0; i < numPins; i++) {
        pinMode(pwmPins[i], OUTPUT);
        analogWriteFrequency(pwmPins[i], pwmFrequency); // Set frequency
        analogWrite(pwmPins[i], dutyCycle); // Set duty cycle
    }
}

void loop() {
    // No need to continuously update, as PWM runs independently after setup
}
