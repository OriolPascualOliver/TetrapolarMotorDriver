
/***************************************************
* Program:   Tetraphase brushless motor driver     *
* Author:   Oriol Pascual                          *
* Date:   07-JUL-21                                *
* rev:    3.0                                      *
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
 ***************************************************/ 
#define DEBUG 1
#define MODE 0

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

// GPIO's
#define CTemp A1    // analog value k to the temp of the coil
#define SPspeed A0  // analog value k to the Set point for the speed
#define CSens A2    // current sensor

#define ind1 3      // DigIn for the absolute position encoder
#define ind2 4      // Ind1:top one, Ind2: middle one, Ind3: bottom one
#define ind3 7

#define PIDErr A4   // Temp controller error
#define SoftStart 8 // Relay to short a high power R seried to the cap to create a soft start
#define Heater 2    // Relay to the controller
#define WaterLevel A3 // Water level Bistae digital sensor
#define Buzz 12     // UI
#define MP 11
#define Led 13

#define C1 10       // Coil outputs
#define C2 9
#define C3 6
#define C4 5




// System Var's
const unsigned long OnDelayPulse = 150; //us to wait pulsed to start the secuence
const int MaxDty = 90;  // duty cicle based on the coil PWM, in 0-254*Vin
const float Kp=2;      // Kp used in the closed loop speed system
const float Ki=2;      // Ki used in the closed loop speed system
const int MaxSpeed = 47; // max steep --> min millis per quarter rev 47 millis/quarte rev = 320RPM
int Dty = 65;           // starting duty cicle
int counter = 0, countMax=3;
int pulse;
int LastErr=0;
bool ON = false;
const int Tmax = 700;  // max temp, value from 0 to 1024
const int speeds[10] = {75, 71, 68, 65, 63, 60, 58, 56, 54, 52}; //speed in millis per quarter of rev [ms]
unsigned long Ti, Tf, aux; 
bool HeaterState = false;
bool RotorState = false;

bool PowerState(){  ////// <-----------------------------------------falla
  #if MODE != 1
  if(!digitalRead(MP)){
    Ti=millis();
    debugln("Flank detected");
    while(!digitalRead(MP)){
      debugln(!digitalRead(MP));
      delay(50);
    }
    Tf=millis();
    aux=Tf-Ti;
    debug("MP detected, Elapsed time: ");
    debugln(aux);
    if(aux>OnDelayPulse){
      debugln("---CHANGE IN POWER STATE---");
      ON=!ON;
      return ON;
      
    }
   }
   debugln("---NO CHANGE IN POWER STATE---");
   return ON;
   #else
    ON=true;
    return true;
   #endif
}

// --- Activates the coils C1, C2, C3, C4
void step1(){
    //debugln("Activating coil num 1");
    while(!digitalRead(ind1) && !digitalRead(ind2)&& !digitalRead(ind3)){
      analogWrite(C1, Dty);
    }
    digitalWrite(C1, LOW);
    CheckStatus();
    
}
int Astep1(){
  /*
   * returns the time it took per 1 rev
   */
   debug("Activating coil num 1 w/ time control, pulse time: ");
    Ti=millis();
    while(!digitalRead(ind1) && !digitalRead(ind2)&& !digitalRead(ind3)){
      analogWrite(C1, Dty);
    }
    
    digitalWrite(C1, LOW);
    Tf=millis();
    debugln(Tf-Ti);
    CheckStatus();
    return int(Tf-Ti);
}

void step2(){
  //debugln("Activating coil num 2");
    while(digitalRead(ind1) && !digitalRead(ind2)&& !digitalRead(ind3)){
      analogWrite(C2, Dty);
    }
    digitalWrite(C2, LOW);
    CheckStatus();
}
void step3(){
  //debugln("Activating coil num 3");
    while(digitalRead(ind1) && digitalRead(ind2)&& !digitalRead(ind3)){
      analogWrite(C3, Dty);
    }
    digitalWrite(C3, LOW);
    CheckStatus();
}
void step4(){
  //debugln("Activating coil num 4");
    while(digitalRead(ind1) && digitalRead(ind2)&& digitalRead(ind3)){
      analogWrite(C4, Dty);
    }
    digitalWrite(C4, LOW);
    CheckStatus();
}

// ---

void HeaterEnable(){
  if(!digitalRead(WaterLevel) && !HeaterState){
    debugln("Waterlevel OK, ---ACTIVATING HEATER RELAY---");
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
  debug("Speed readed, SetSpeed: ");
  debugln(SPd);
  return SPd;
}


int CheckPosition(){
  /*
  Detects the position of the rotor in 1/4 of rotation, returns the nearest coil in the desired direction
  OUTPUT INT
  */
  bool I1 = digitalRead(ind1);
  bool I2 = digitalRead(ind2);
  bool I3 = digitalRead(ind3);
  
  debug("Checking position, IND1, IND2, IND3: ");
  debug(I1);
  debug(I2);
  debugln(I3);
  debug("Facing Coil num: ");
  
  
  if(!I1 && !I2 && !I3){
    debugln(1);
    return 1;
  }
  else if(I1 && !I2 && !I3){
    debugln(2);
    return 2;
  }
  else if(I1 && I2 && !I3){
    debugln(3);
    return 3;
  }
  else{
    debugln(4);
    return 4;
  }
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
  //debugln("-- TEMP OK --");
}

void err1(){
  /*
  overheated coil error
  */
  debugln("### ENTERING ERROR1 MODE ###");
  digitalWrite(C1, LOW);
  digitalWrite(C2, LOW);
  digitalWrite(C3, LOW);
  digitalWrite(C4, LOW);
  digitalWrite(Heater, LOW);
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
}
void err2(){
  /*
  rotor problem
  */
  debugln("### ENTERING ERROR2 MODE ###");
  digitalWrite(C1, LOW);
  digitalWrite(C2, LOW);
  digitalWrite(C3, LOW);
  digitalWrite(C4, LOW);
  digitalWrite(Heater, LOW);
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
}

void GoToStart(){
  /*
  goes to a known position to start in a known direction
  */
  debugln("--- GOING TO START POSITION ---");
  digitalWrite(SoftStart, HIGH);
  switch (CheckPosition()){

    case 1:
      debugln(">Case1");
      step1();
      break; 
    case 2:
      debugln(">Case2");
      step2();
      step3();
      step4();
      step1(); 
      break;
    case 3:
      debugln(">Case3");
      step3();
      step4();
      step1();
      break;
    case 4:
      debugln(">Case4");
      step4();
      step1();
      break;
  }

}



int GetDty(int Pv){
  debugln("--- SPEED P Control ---");
  if(Pv == 0 || Pv == 1){
    return 5;
  }
  
  else if(MaxSpeed > Pv){
    debugln("#### OVERSPEED DETECTED ####");   
    err2();
  }
   
  int E, Sp=SetSpeed();
  
  E=int((Sp - Pv)*Kp +LastErr*Ki);
  debug("> Calculated Duty cicle: ");
  debugln(E);
  LastErr=E;
  if(E>MaxDty){
    debug("> Set Duty cicle: ");
    debugln(MaxDty);
    return MaxDty;
  }
  else if(E<5){
    debug("> Set Duty cicle: ");
    debugln(5);
    return 5;
  }
  return E;
 
}

void CheckStatus(){
  //debugln("--- Checking internal stats ---");
  #if MODE != 3 
  bool PrivateAux=PowerState();
  CheckTemp();
  if((digitalRead(PIDErr) && HeaterState) || digitalRead(WaterLevel)){
    debugln("#### HEATER ERROR #### check water level or pid controller");
    debug("PIDErr, HeaterState, Waterlevel: ");
    debug(digitalRead(PIDErr));
    debug(HeaterState);
    debugln(digitalRead(WaterLevel));
    
    debugln("Turning OFF water heater :(");
    digitalWrite(Heater, LOW);
    err1();
  }
  #endif
}

void Run(){
  if(counter == countMax){
        pulse=Astep1();
        Dty = GetDty(pulse);
        counter = 0;
      }
      else{
        step1();
        }
      step2();
      step3();
      step4();
      //debug(">counter: ");
      //debugln(counter);
      
    counter++;
  
}
void setup(){
  pinMode(CTemp, INPUT);
  pinMode(SPspeed, INPUT);
  pinMode(CSens, INPUT);
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
  digitalWrite(Led, HIGH);
  Serial.begin(9600);

  CheckTemp();
  HeaterEnable();
  debugln(">>> VOID SETUP OK");
}
void loop(){
  debugln("Hi");
  if(PowerState()){
    debugln(">>> POWER ON <<<");
    tone(Buzz, 750, 1000);
    delay(500);
    digitalWrite(Led, LOW);
    #if MODE != 2
    GoToStart();
    #endif
    
    Dty = 80;
    while(ON){
      Run();
      }  
    }
    else{
      delay(5);
    }
}
