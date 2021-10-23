/***************************************************
* Program:   Tetraphase brushless motor modulator  *
* Author:   Oriol Pascual                          *
* Date:   18-OCT-21                                *
* rev:    1.0, Release: 23/10/2021, Stat: TEST     *
****************************************************/
//Code for arduino ProMini!

#include <PWM.h>
//https://github.com/terryjmyers/PWM

// GPIO's
#define Update   2
#define Err         6
#define LED         13
#define PWM         3
#define Data A0
#define DtyMin 5
#define DtyMax 100


uint32_t freq = 16000;
uint32_t MinPeriod = 10; //max speed period in ms
int Dty;

/*int GetPulse(){
    while(!digitalRead(SpeedSens)){
    }
    Ti=millis();
    while(digitalRead(SpeedSens)){
   
    }
    Tf=millis();
    return int(Tf-Ti);
}
*/

void setup(){
    pinMode(Err, OUTPUT);
    pinMode(LED, OUTPUT);
    pinMode(Data, INPUT);
    pinMode(Update, INPUT);
    //Serial.begin(9600);
    //attachInterrupt(digitalPinToInterrupt(PIN), Funcio, RISING); //volatile a les variables

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
    
}
void loop(){
    if(!digitalRead(Update)){
        Dty=map(analogRead(Data), 0, 1023, DtyMin, DtyMax);
              
    }
 pwmWrite(PWM, Dty);


}
