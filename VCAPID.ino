/**
 * This script outlines a PID controller for a voice coil actuator in the SmithVent to provide PEEP
 * The PID input is a pressure reading from the expiratory line
 * The PID output is current that will be an input to the VCA
 * This code was not tested with the VCA or ventilator and is only a pseudocode for testing and tuning the PID
*/
// ------------------Libraries------------------
#include <PID_v1.h>

//--------------Initialize values--------------
//Pin variables
const int SV1_CONTROL = 22;           //relay pin that controls SV1 (air)
const int SV2_CONTROL = 24;           //relay pin that controls SV2 (O2)
const int SV3_CONTROL = 5;            //proportional valve pin SV3 (insp)
const int SV4_CONTROL = 26;           //relay pin that controls SV4 (exp)
const int CURRENT = 1;                //pin connected to current input for VCA, will need to determine
const int CURRENTREAD = 2;            //pin connected to the current read, will need to determine
const int PRESSURE_RESERVOIR = A5;    //pressure sensor in reservoir
const int PRESSURE_EXP = A7;          //pressure sensor in expirtory line

//User determined variables
double breathtime = 3000; //time that expiration state/inspiration state runs for before switching
double VCAsetpoint = 15;  //desired PEEP from user input cmH2O

//state machine variables
double ExpP;                            //expiratory line pressure
double PEEPerror;                       //error between setpoint and current PEEP
double VCAinput;                        //input current to the VCA
double current;                         //current read from arduino/other current source 
double ExpStartTime;                    //utilized to determine how long expiration has been running
double InspStartTime;                   //utilized to determine how long inspiration has been running
enum States {Expiration, Inspiration};  //declare state machine cases as variables with type States 
States state;                           //declare the variable state with the type States


//-------------------PID variables + declaration--------------------
double VCAPID_Input, VCAPID_Output, VCAPID_Setpoint;   //declare PID variables
double VCAKp=0.1, VCAKi=0, VCAKd=0;                   //Set VCA gain values
PID VCAPID(&VCAPID_Input, &VCAPID_Output, &VCAPID_Setpoint, VCAKp, VCAKi, VCAKd, DIRECT);//Set up VCA PID

const double OUTPUT_MAX = 1;        //PID output ranges from 0-1 
const double OUTPUT_MIN = 0;        //corresponds to min 0 Am[ and Max 1 Amp
const int SAMPLE_TIME = 100;
unsigned long PID_interval = 100;   //Update PID on this interval (ms)


//-------------------Sensor readings + equations--------------------
double Pressure(int sensor) { //calculate and return expiratory/inspiratory pressure from esensor in PSI
  int R = analogRead(sensor);                    // read the voltage
  static const float mBarTocmH2O = 1.01972;
  static const float Pmin = -163.155 * mBarTocmH2O;   // pressure max in cmH2O
  static const float Pmax = 163.155 * mBarTocmH2O;    // pressure min in cmH2O
  static const float Prange = Pmax - Pmin;            
  const unsigned long Vsupply     = 5000;                    // voltage supplied, mv
  const unsigned long sensorMin   = 1023UL * 500 / Vsupply;  // Sensor value at 500 mv
  const unsigned long sensorMax   = 1023UL * 4500 / Vsupply; // Sensor value at 4500 mv
  const unsigned long sensorRange = sensorMax - sensorMin;
  double P = (R - sensorMin) * (Prange / sensorRange) + Pmin; //convert to pressure in cmH2O
  return P;
}

//-------------------PID Functions--------------------
double runPID(){  //calculates PID 
  VCAPID.SetMode(AUTOMATIC);              //Turn on PID
  VCAPID_Setpoint = VCAsetpoint;          //Define the setpoint (cmH2O) of the PID 
  VCAPID_Input = Pressure(PRESSURE_EXP);  //Define the input of the PID as current PEEP
  VCAPID.Compute();
  return VCAPID_Output;
}

void initializePID(double OUTPUT_MIN, double OUTPUT_MAX, double SAMPLE_TIME){
  VCAPID.SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);  //set PID limits
  VCAPID.SetSampleTime(SAMPLE_TIME);               
}

//-------------------State Machine--------------------
void StateMachine(){  //used to switch between exhalation and inhalation
  switch (state) {
    case Expiration:
      ExpP = Pressure(PRESSURE_EXP);      //update expiratory line pressure value
      PEEPerror = VCAsetpoint - ExpP;     //take error between setpoint and current PEEP
            
      if (millis() - ExpStartTime >= breathtime){   //if expiration state has been running for desired time
        state = Inspiration;                        //set state to expiration digital
        InspStartTime = millis();                   //set time we start expiration state  
        digitalWrite(SV4_CONTROL, HIGH);            //close SV4
      }else if (PEEPerror > 0 && abs(PEEPerror) > 2){ //if the setpoint is larger than the existing PEEP and error > x
      //error > x makes sure current is not adjusting for small pressure differences
      //need to determine x (currently set as 2)
        VCAinput = runPID();                        //run the PID and set the current to the PID determined value
        analogWrite(CURRENT, VCAinput);             //set the current as the PID output value
      }
      break;
  
      
    case Inspiration:
      ExpP = Pressure(PRESSURE_EXP);    //take pressure reading from expiratory line            
      PEEPerror = VCAsetpoint - ExpP;   //take error between current PEEP + desired PEEP
      
      if (millis() - InspStartTime >= breathtime){    //if inspiration state has been running for desired time
        state = Expiration;                           //set state to expiration digitalWrite(SV4_CONTROL, HIGH);                 
        ExpStartTime = millis();                      //Set time we start expiration state
        digitalWrite(SV4_CONTROL, LOW);               //open SV4
      }else if (PEEPerror < 0){                       //if the setpoint is smaller than the existing PEEP
        analogWrite(CURRENT, 1);                      //set the current to 1
        delay (100);                                  //allow for action to happen
        analogWrite(CURRENT, 0);                      //set the current to 0
      }else if (PEEPerror > 0 && abs(PEEPerror) > 2){ //if the setpoint is larger than the existing PEEP and error > x
      //need to determine x (currently set as 2)
        VCAinput = runPID();                          //run the PID and set the current to the PID determined value
        analogWrite(CURRENT, VCAinput);               //set the current as the PID output value
      }
      break;
  }
}

void report(double VCAPID_Output, States state) { 
  ExpP = Pressure(PRESSURE_EXP);      //take pressure reading from expiratory line
  current = analogRead(CURRENTREAD);  //take current reading from arduino/current source
  Serial.print(millis());             //time
  Serial.print("\t");
  Serial.print(state);                //state
  Serial.print("\t");
  Serial.print(current);              //expiratory line pressure
  Serial.print("\t");
  Serial.print(VCAPID_Output);        //PID output (current)
  Serial.print("\t");
  Serial.print(VCAsetpoint);          //setpoint set as global variable
  Serial.print("\t");
  Serial.println(ExpP);               //existing O2 PEEP value                              
}

void setup() {
  delay(2000); //allows operator to start serial monitor
  pinMode(SV1_CONTROL, OUTPUT); //set air valve as an output
  pinMode(SV2_CONTROL, OUTPUT); //set O2 valve as an output
  pinMode(SV3_CONTROL, OUTPUT); //set proportional valve as an output
  pinMode(SV4_CONTROL, OUTPUT); //set expiratory valve as an output 
  pinMode(CURRENT, OUTPUT);     //set current from source as an output
  
  pinMode(CURRENTREAD, INPUT);  //set current being read from source as an input
  pinMode(PRESSURE_EXP, INPUT); //set expiratory pressure sensor as an input
  initializePID(OUTPUT_MIN, OUTPUT_MAX, SAMPLE_TIME); //initialize O2PID values above
  Serial.begin(115200);
  Serial.println("time\tState\tCurrent\tPIDout\tSetpoint\tExpP");
  States state = Expiration;    //initially set state to expiration
  ExpStartTime = millis();      //set time we start expiration state
}

void loop() {
  StateMachine();               //run state machine
  report(VCAPID_Output, state); //run report() to print values in serial montor
}
