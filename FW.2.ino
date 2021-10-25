/***************************************************
* Program:   Tetraphase brushless motor driver     *
* Author:   Oriol Pascual                          *
* Date:   07-JUL-21                                *
* rev:    3.5, Release: 21/10/2021, Stat: STABLE   *
****************************************************/

/*************************************************** 
*                  --DEBUGGING-- 
*      Set DEBUG to 0 for disable Serial Debugging
*                   1 for enable Serial Debugging
*                  
*      Set MODE to  0 for normal operation
*                   1 for power right up
*                   2 for turn right away
*                   3 for disable error checks
*      Set ENsound to 1 to enable buzzer
*                     0 to disable buzzer
*                     
 ***************************************************/ 
#define DEBUG 0
#define MODE 0
#define ENsound 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

// GPIO's
#define SPspeed   A0   // analog value k to the Set point for the speed
#define CTemp     A1     // analog value k to the temp of the coil
#define ModulatorErr A2 // Sever modulator error W CanPask
#define WaterLevel A3  // Water level digital sensor
#define PIDErr    A4     // Temp controller error + Err for CanPask Comunication
#define ind1      A5    // DigIn for the absolute position encoder
#define Update    A6   // Update for CanPask


// Pin 0,1 dedicated for UART comunication
#define Heater    2      // Relay to the PID external controller
#define PWMOut    3     // Data out for CanPask
#define ind2      4    // Ind1:top one, Ind2: middle one, Ind3: bottom one
#define C4        5
#define C3        6
#define ind3      7
#define Led       8
#define C2        9   
#define C1        10      // Coil outputs
#define MP        11 
#define Buzz      12    // UI
#define SoftStart 13   // Relay to short a high power R seried to the cap to create a soft start for the first seconds
        


// System Var's
const unsigned long OnDelayPulse = 1500; //ms to wait pulsed to start the secuence
const uint16_t MaxDty = 200, MinDty = 5;  // duty cicle based on the coil PWM, in 0-254*Vin
const float Kp=0.8;      // Kp used in the closed loop speed system
const float Ki=0.1;      // Ki used in the closed loop speed system
const uint16_t MaxSpeed = 188; // max steep --> min millis per quarter rev 47 millis/quarte rev = 320RPM
uint8_t Dty = 170;           // starting duty cicle
uint8_t counter = 0, countMax = 2;
int LastErr=0;
uint8_t MaxSpeedCount=0;
bool ON = false;
const uint16_t Tmax = 700;  // max temp, value from 0 to 1024
const int speeds[10] = {75, 71, 68, 65, 63, 60, 58, 56, 54, 52}; //speed in millis per quarter of rev [ms]
uint8_t Spd_multiplier = 4;
unsigned long Ti, Tf, aux; 
bool HeaterState = false;
bool RotorState = false;

/*
TCCR1 = 0xB2;
ICR1 = 0x01FF;
OCR1A = pwm (analog/2)
*/




bool PowerState(){
  #if MODE != 1
  if(!digitalRead(MP)){
    Ti=millis();
    debugln("> UP Flank detected");
    while(!digitalRead(MP)){
      delay(50);
    }
    Tf=millis();
    aux=Tf-Ti;

    if(aux>OnDelayPulse){

      tone(Buzz, 750, 1000);
      ON=!ON;
      digitalWrite(Led, ON);
      return ON;
      
    }
   }

   return ON;
   #else
    ON=true;
    return true;
   #endif
} 

void CheckStatus(){
  //debugln("--- Checking internal stats ---");
  #if MODE != 3 
  bool PrivateAux=PowerState();
  CheckTemp();
  if((digitalRead(PIDErr) && HeaterState) || digitalRead(WaterLevel)){
    debugln("#### HEATER ERROR ####");
    debugln("Turning OFF water heater :(");
    digitalWrite(Heater, LOW);
    err1();
  }
  #endif
}

int Aturn(){
  debugln(">>>> Aturn");
  Ti=millis();

  
    digitalWrite(C1, HIGH);
    delayMicroseconds(5000);
    while(!digitalRead(ind1)){ //MISSING ANOTHER CONDITION!!!!
   
    }
    digitalWrite(C1, LOW);
    
    digitalWrite(C2, HIGH);
    delayMicroseconds(5000);
    while(!digitalRead(ind2) && digitalRead(ind1)){
 
    }
    digitalWrite(C2, LOW);

    digitalWrite(C3, HIGH);
    delayMicroseconds(5000);
    while(!digitalRead(ind3) && digitalRead(ind2)){
      
    }
    digitalWrite(C3, LOW);

    digitalWrite(C4, HIGH);
    debugln("C4");
    delayMicroseconds(5000);
    while(digitalRead(ind1) || digitalRead(ind2)){
      
    }
    digitalWrite(C4, LOW);
    Tf=millis();

    debug("Time: ");
    debugln(Tf-Ti);
    CheckStatus();
    return int(Tf-Ti);

}


void step1(){
    digitalWrite(C1, HIGH);
    debugln("C1");
    delayMicroseconds(5000);
    while(!digitalRead(ind1)){

    }
    digitalWrite(C1, LOW);
}
void step2(){
    digitalWrite(C2, HIGH);
    debugln("C2");
    delayMicroseconds(5000);
    while(!digitalRead(ind2) && digitalRead(ind1)){

    }
    digitalWrite(C2, LOW);
}
void step3(){
    digitalWrite(C3, HIGH);
    debugln("C3");
    delayMicroseconds(5000);
    while(!digitalRead(ind3) && digitalRead(ind2)){

    }
    digitalWrite(C3, LOW);
}
void step4(){
    digitalWrite(C4, HIGH);
    debugln("C4");
    delayMicroseconds(5000);
    while(digitalRead(ind1) || digitalRead(ind2)){

    }
    digitalWrite(C4, LOW);
}
void multipleSteps(int coil){
  if(coil == 1){
    
  }
  
}



void HeaterEnable(){
  if(!digitalRead(WaterLevel) && !HeaterState){
    //debugln("Waterlevel OK, ---ACTIVATING HEATER RELAY---");
    HeaterState=true;
    digitalWrite(Heater, HIGH);
  }  
 }

 
int SetSpeed(){
  /*
  Reads the SetPSpeed and outputs a mills/rev value
  OUTPUT INT
  */
  int SPd = speeds[map(analogRead(SPspeed), 0, 1023, 0, 9)];
  debug("SP: ");
  debugln(SPd*Spd_multiplier);
  return SPd*Spd_multiplier;
}


int CheckPosition(){
  /*
  Detects the position of the rotor in 1/4 of rotation, returns the nearest coil in the desired direction
  OUTPUT INT
  */
  bool I1 = digitalRead(ind1);
  bool I2 = digitalRead(ind2);
  bool I3 = digitalRead(ind3);
  
  
  if(!I1 && !I2 && !I3){
    return 1;
  }
  else if(I1 && !I2 && !I3){
    return 2;
  }
  else if(I1 && I2 && !I3){
    return 3;
  }
  else{
    return 4;
  }
}

void CanPask(int value){
  analogWrite(PWMOut, value);
  debugln("CanPaskiiiiiii:");
  Serial.write(int(char(value)));
  Serial.println();
}

void CheckTemp(){
  /*
  Max temp error
    SELF
  */
  // --- NOTE ---
  // for PTC change the sign of the operator
  if(analogRead(CTemp)>Tmax){
    debugln("### TEMP ERR ###");
    err1();
  }
}

void err1(){
  /*
  overheated coil error
  */
  debugln("### ENTERING ERROR1 MODE ### :(");
  digitalWrite(C1, LOW);
  digitalWrite(C2, LOW);
  digitalWrite(C3, LOW);
  digitalWrite(C4, LOW);
  digitalWrite(Heater, LOW);
  #if ENsound == 1
  for(;;){
    digitalWrite(Led, HIGH);
    tone(Buzz, 2000);
    delay(500);
    noTone(Buzz);
    delay(500);
    digitalWrite(Led, LOW);
    tone(Buzz, 2000);
    delay(500);
    tone(Buzz, 2000);
    delay(500);
  }
  #else
  for(;;){
    digitalWrite(Led, HIGH);
    delay(1000);
    digitalWrite(Led, LOW);
    delay(100);
  }
  #endif
}
void err2(){
  /*
  rotor problem
  */
  debugln("### ENTERING ERROR2 MODE ### :(");
  digitalWrite(C1, LOW);
  digitalWrite(C2, LOW);
  digitalWrite(C3, LOW);
  digitalWrite(C4, LOW);
  digitalWrite(Heater, LOW);
  #if ENsound == 1
  for(;;){
    digitalWrite(Led, HIGH);
    tone(Buzz, 2000);
    delay(1000);
    tone(Buzz, 2000);
    delay(1000);
    digitalWrite(Led, LOW);
    tone(Buzz, 2000);
    delay(1000);
    tone(Buzz, 2000);
    delay(1000);
  }
  #else
  for(;;){
    digitalWrite(Led, HIGH);
    delay(2000);
    digitalWrite(Led, LOW);
    delay(2000);
  }
  #endif
  
}

void GoToStart(){
  /*
  goes to a known position to start in a known direction
  */
  //debugln("--- GOING TO START POSITION ---");

  digitalWrite(SoftStart, HIGH);
  CanPask(Dty);
  switch (CheckPosition()){

    case 1:
      step1();
      break; 
    case 2:
      step2();
      step3();
      step4();
      step1(); 
      break;
    case 3:
      step3();
      step4();
      step1();
      break;
    case 4:
      step4();
      step1();
      break;
  }

}


uint8_t GetDty(int Pv){
  debugln("--- SPEED PI Control ---");
  //ToRpm(Pv);
  if(Pv == 0 || Pv == 1){
    if(LastErr>MaxDty){
      counter --;
      debug("PID: ");
      debugln(MaxDty);
      analogWrite(PWMOut, MaxDty);
      return(MaxDty);
      
    }
    else{
      counter --;
      debug("PID: ");
      debugln(MinDty);
      analogWrite(PWMOut, MinDty);
      return(MinDty);
    }
  }
  
  
  else if(MaxSpeed > Pv && MaxSpeedCount>=100){
    debugln("#### OVERSPEED DETECTED ####");   
    err2();
  }
  else if(MaxSpeed > Pv){
    MaxSpeedCount ++;
    debug("PID: ");
    debug(MinDty);
    analogWrite(PWMOut, MinDty);
    
  }
  else{

        int E, Sp=SetSpeed();
    
      E=int((Pv - Sp)*Kp +LastErr*Ki);
        debug("Error: ");
        debugln(E);
        if(E>MaxDty){
          debug("PID: ");
          debugln(MaxDty);
          counter=0;
          LastErr=MaxDty;
          analogWrite(PWMOut, MaxDty);
          return(MaxDty);
        }
        else if(E<MinDty){
          debug("PID: ");
          debugln(MinDty);
          counter=0;
          LastErr=MinDty;
          analogWrite(PWMOut, MinDty);
          return(MinDty);
        }
        else{
        counter=0;
        debug("PID: ");
        debug(E);
        LastErr=E;
        analogWrite(PWMOut, E);
        return(E);
      }
   }
}


void Run(){
  if(counter < countMax){
        step1();
        step2();
        step3();
        step4();

      }
  else{
        CanPask(GetDty(Aturn()));
        }     
    counter++;
  
}
void setup(){
  pinMode(CTemp, INPUT);
  pinMode(SPspeed, INPUT);
  pinMode(ind1, INPUT);
  pinMode(ind2, INPUT);
  pinMode(ind3, INPUT);
  pinMode(Buzz, OUTPUT);
  pinMode(MP, INPUT_PULLUP);
  pinMode(Led, OUTPUT);
  pinMode(PIDErr, INPUT_PULLUP);
  pinMode(C1, OUTPUT);
  pinMode(C2, OUTPUT);
  pinMode(C3, OUTPUT);
  pinMode(Heater, OUTPUT);
  pinMode(WaterLevel, INPUT_PULLUP);
  pinMode(Update, OUTPUT);
  digitalWrite(PWMOut, OUTPUT);
  digitalWrite(Led, HIGH);
  digitalWrite(SoftStart, HIGH);
  
  /*for(;;){
    digitalWrite(Led, HIGH);
    digitalWrite(Heater, HIGH);
    digitalWrite(SoftStart, HIGH);
    delay(1000);
    digitalWrite(Led, LOW);
    digitalWrite(Heater, LOW);
    digitalWrite(SoftStart, LOW);
    delay(1000);
    
  }
    */    
        
  delay(500);
  digitalWrite(Led, LOW);
  Serial.begin(9600);

  CheckTemp();
  HeaterEnable();
  debugln(">>> VOID SETUP OK");
}
void loop(){
  if(PowerState()){
    debugln(">>> POWER ON <<<");
    delay(500);
    //digitalWrite(Led, HIGH);
    #if MODE != 2
    GoToStart();
    #endif
    
    Dty = 80;
    while(ON){
      Run();
      CheckStatus();
      }  
    }
    else{
      delay(5);
    }
}
