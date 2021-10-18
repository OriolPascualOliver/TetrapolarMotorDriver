/***************************************************
* Program:   Tetraphase brushless motor modulator  *
* Author:   Oriol Pascual                          *
* Date:   18-OCT-21                                *
* rev:    1.0, Release: 18/10/2021, Stat: TEST     *
****************************************************/
//Code for arduino ProMini!

#include <PWM.h>
//https://github.com/terryjmyers/PWM

// GPIO's
#define Update      4
#define Data        A3
#define SpeedSens   5
#define Err         6
#define LED         13
#define PWM         3

uint32_t freq = 25000;
uint32_t MinPeriod = 10; max speed period in ms
int Duty;

void setup(){
    pinMode(Err, OUTPUT);
    pinMode(LED, OUTPUT);
    pinMode(SpeedSens, INPUT);
    pinMode(Data, INPUT);
    pinMode(Update, INPUT);


    InitTimersSafe(); 
    bool success = SetPinFrequencySafe(PWM, freq);
    delay(500);
  
    if(!success) {
      digitalWrite(Err, LOW);    
    }
    else{
        digitalWrite(LED, HIGH);
        delay(1500);
        digitalWrite(LED, LOW);

    }
    while(!digitalRead(Update)){
        delayMicroseconds(1000);
    }
    digitalWrite(LED, HIGH);
    delay(150);
    digitalWrite(LED, LOW);
    delay(150);
    digitalWrite(LED, HIGH);
    delay(150);
    digitalWrite(LED, LOW);


}
void loop(){
    if(!digitalRead(Update)){
        Duty=analogRead(Data);
        pwmWrite(PWM, Dty);





    }



}
