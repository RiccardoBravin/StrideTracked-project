/*
  Contains the function that is called by the C++ code to visualize the result of inference using 
  the Arduino's built-in LED. 
  Also contains the code to read the shield button and switch between inference and training mode.
*/


#include <algorithm>

#include "Arduino.h"
#include "constants.h"
#include "output_handler.h"


void class_to_led(int class_label, bool log_silence = false){

  // The pin of the Arduino's built-in LED
  static const int led_r = 22;
  static const int led_g = 23;
  static const int led_b = 24;    

  // Track whether the function has run at least once
  static bool initialized = false;

  // Do this only once
  if (!initialized) {
    // Set the LED pin to output
    pinMode(led_r, OUTPUT);
    pinMode(led_g, OUTPUT);
    pinMode(led_b, OUTPUT);

    digitalWrite(led_r, LOW);
    digitalWrite(led_r, LOW);
    digitalWrite(led_r, LOW);

    initialized = true;
  }

  //could probably be changed to analogWrite if we want to use PWM and give values from 0 to 255

  if(class_label == 1){
    digitalWrite(led_r, HIGH);
    digitalWrite(led_r, LOW);
    digitalWrite(led_r, LOW);
  }else if(class_label == 2){
    digitalWrite(led_r, LOW);
    digitalWrite(led_r, HIGH);
    digitalWrite(led_r, LOW);
  }else if(class_label == 3){
    digitalWrite(led_r, LOW);
    digitalWrite(led_r, LOW);
    digitalWrite(led_r, HIGH);
  }else if(class_label == 4){
    digitalWrite(led_r, HIGH);
    digitalWrite(led_r, HIGH);
    digitalWrite(led_r, LOW);
  }else if(class_label == 5){
    digitalWrite(led_r, HIGH);
    digitalWrite(led_r, LOW);
    digitalWrite(led_r, HIGH);
  }else if(class_label == 6){
    digitalWrite(led_r, LOW);
    digitalWrite(led_r, HIGH);
    digitalWrite(led_r, HIGH);
  }else if(class_label == 0 && log_silence){
    digitalWrite(led_r, HIGH);
    digitalWrite(led_r, HIGH);
    digitalWrite(led_r, HIGH);
  }
}

static int protothreadBlinkLED(struct pt *pt, bool &training)
{
  static unsigned long lastTimeBlink = 0;
  static const int PW_LED_PIN = 25;   //the number of the LED pin
  PT_BEGIN(pt);
  while(training) {
    lastTimeBlink = millis();
    PT_WAIT_UNTIL(pt, millis() - lastTimeBlink > 250);
    digitalWrite(PW_LED_PIN, HIGH);
    lastTimeBlink = millis();
    PT_WAIT_UNTIL(pt, millis() - lastTimeBlink > 250);
    digitalWrite(PW_LED_PIN, LOW);
  }
  PT_END(pt);
}
