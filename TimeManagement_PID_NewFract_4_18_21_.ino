/**
 * This script outlines a system to measure and control the oxygen concentration in the reservoir
 * The PID input is an O2 reading from the inspiratory line
 * The PID output is a fraction that will multiplied by the fill time to calculate time O2 and air valves will be open for
 * This code was tested with the oxygen sensor and ventilator, but the fill time was incorrect
 * The fill time did not allow the reservoir to reach 25 psi (desired) using the topuptime() function
 * The (if resP<=15) conditon allowed the reservoir to stay above 15 psi
 * The system was shown to be able to control concentrations between 21% and ~100% oxygen
*/

// ------------------Libraries------------------
#include <PID_v1.h>

//--------------Initialize values--------------
//Pin variables
const int SV1_CONTROL = 22;         //relay pin that controls SV1 (air)
const int SV2_CONTROL = 24;         //relay pin that controls SV2 (O2)
const int SV3_CONTROL = 5;          //proportional valve pin SV3 (insp)
const int SV4_CONTROL = 26;         //relay pin that controls SV4 (exp)
const int O2_SENSOR = A8;           //O2 sensor pin
const int PRESSURE_RESERVOIR = A5;  //pressure sensor in reservoir


//User determined variables
double breathtime = 3000; //time that expiration state/inspiration state runs for before switching
int o2setpoint = 100;     //desired o2 concentration from user input

// oxygen sensor variables
float oxygenConcentration = 0;  //variable to store the O2 value read, may replace with o2input
double o2time = 0;              //time O2 valve needs to be left open for
double airtime = 0;             //time air valve needs to be left open for

//state machine variables
double resP;              //reservoir pressure found from pressure sensor 
double o2input;           //O2 concetration in inspiratory line found from O2 sensor
double error;             //error between O2 setpoint and O2 input
double filltime;          //time to fill tank, calculated from equation
double o2fraction;        //fraction of topup time that O2 valve should be open
double airfraction;       //fraction of topup time that air valve should be open
double AirStartT;         //utilized to determine how long air has been running
double O2StartT;          //utilized to determine how long O2 has been running
double ExpStartTime;      //utilized to determine how long expiration has been running
double InspStartTime;     //utilized to determine how long inspiration has been running
enum States {ExpirationNull, ExpirationWait, PIDtime, Timing, ExpirationValves, Inspiration};
//declare state machine cases as variables with type States 
States state;             //declare the variable state with the type States

//-------------------PID variables + declaration--------------------
double O2PID_Input, O2PID_Output, O2PID_Setpoint;                                 //declare PID variables
double O2Kp=0.2, O2Ki=.1, O2Kd=0;                                                 //set O2 gain values
PID O2PID(&O2PID_Input, &O2PID_Output, &O2PID_Setpoint, O2Kp, O2Ki, O2Kd, DIRECT);//set up O2 PID

const double OUTPUT_MAX = 1;            //PID output ranges from 0-1
const double OUTPUT_MIN = 0;
const int SAMPLE_TIME = 100;
unsigned long PID_interval = 100;       //update PID on this interval (ms)


//-------------------Sensor readings + equations--------------------
float readOxygenSensor() {//calculate and return concentration from oxygen sensor
  analogReference(INTERNAL1V1);                                       //reference internal reference -
  analogRead(O2_SENSOR);                                              //discard first reading
  double O2SensorReading = (((double)analogRead(O2_SENSOR))*5)/1023;  //set O2 reading pin, converitng digital output to analog (voltage)
  analogReference(DEFAULT);                                           //switch back to default reference
  analogRead(O2_SENSOR);                                              //discard first reading
  double O2concentration = ((113.6)*(O2SensorReading)) + 7.6892;      //equation found from testing, calculate concentration from voltage
  return O2concentration;                                             //returns calculated O2 concetration value
}

double respressure(){ //calculate and return reservoir pressure from sensor in PSI
  double pv = analogRead(PRESSURE_RESERVOIR);             //read pressure sensor pin
  double pressureCM = 70.307*100*(5.0*pv/1023-0.25)/4.5;  //convert volts to cm h2o
  double PSI = 0.014223343307054 * pressureCM;            //convert cm h2o to psi 
  return PSI;
}

double topuptime(){//calculate and return time to fill from current pressure to upper pressure limit
  //Not giving correct filltime
  double gamma = 1.4;                                       //Cp/Cv
  double Psource = 50+14.7;                                 //psia air pressure coming in
  double Pinitial = respressure()+14.7;                     //psia initial air pressure - from reservoir pressure sensor repressure()
  double Tsource  = 20;                                     //deg C temperature of air coming in
  double Tinitial = 20;                                     //deg C initial temperature of tank
  double V = .0012;                                         //m^3 volume of tank
  double At = (3.14/4)*pow(0.125,2)*1/1550;                 //m^2 cross sectional area of throat, assuming diameter of 1/8"
  double as = 20.05*pow((273+Tsource),0.5);                 //speed of sound in air at Tsource
  double tchar = V/(At*as);                                 //charateristic time
  double Piplus = Pinitial/Psource;                         //dimensionless initial pressure
  double Tiplus = Tinitial/Tsource;                         //dimensionless initial temperature
  double Pfill = 25+14.7;                                   //fill pressure absolute - CAN CHANGE TO OUR MAX
  double Pplusfill = Pfill/Psource;                         //dimensionless fill pressure
  double tplusfill = (Pplusfill - Piplus)/(Tiplus*(pow(((gamma+1)/2),(-1*(gamma + 1)/(2*gamma - 2))))); 
  double filltime = tplusfill*tchar*1000;                   //actual time to reach fill pressure ms
  return filltime;                                          //returns time to fill tank, calculated from equation
}

double fraction(){//calculate and return fraction of the gas mixture entering the reservoir should be oxygen
  double pi = respressure();                            //initial pressure = pressure in reservoir
  double pf = 25;                                       //pressure we want/final
  double ciFrac = 0.0127 * readOxygenSensor() - 0.266;  //current fraction of o2 in mix
  double cfFrac= 0.0127 * o2setpoint - 0.266;           //desired fraction of o2 in mix
  o2fraction = (cfFrac*pf-ciFrac*pi)/(pf-pi);           //fraction of o2 to be added
  return o2fraction;
}

//-------------------PID Functions--------------------
double runPID(){//calculate and PID output
  O2PID.SetMode(AUTOMATIC);         //Turn on PID
  O2PID_Setpoint = o2setpoint;      //Define the setpoint (O2 concentratoin) of the PID 
  O2PID_Input = readOxygenSensor(); //Define the input of the PID as current O2 concentration
  O2PID.Compute();
  return O2PID_Output;
}

void initializePID(double OUTPUT_MIN, double OUTPUT_MAX, double SAMPLE_TIME){
  O2PID.SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  O2PID.SetSampleTime(SAMPLE_TIME);
}

//-------------------State Machine--------------------
void StateMachine(){
  switch (state) {
    case ExpirationNull:
      analogWrite(SV3_CONTROL, 0);                    //close SV3
      resP = respressure();                           //update reservoir pressure value
      o2input = readOxygenSensor();                   //update oxygen concentration value
      error = o2setpoint-o2input;                     //take error between setpoint and existing concentration
      
      if (millis() - ExpStartTime > breathtime){      //if expiration state has been running for desired time
        state = Inspiration;                          //go to the inspiration state
        InspStartTime = millis();                     //set time we start inspiration state
      } else if (abs(error) <= 20 && abs(error) > 0 && resP < 25){//if the error < 20, not 0 and reservoir pressure is below 25
        state = PIDtime;                              //set state to PIDtime 
      } else if (abs(error) > 20 && resP < 25){       //if the error > 20 and reservoir pressure is below 25
        state = Timing;                               //set state to timing 
      } else if (resP <=15) {                         //if reservoir pressure is <=15
        resP = respressure();                         //update reservoir pressure value
        filltime = topuptime()/2;                     //calculate new fill time
        o2fraction = 0.0127 * o2setpoint - 0.266;     //calculate fraction of O2 needed to find correct concentration 
        airfraction = 1 - o2fraction;                 //calculate fraction of air needed to find correct concentration 
        o2time = o2fraction * filltime;               //calculate time for O2 valves to open
        airtime = airfraction * filltime;             //calculate time for air valves to open
        state = ExpirationValves;                     //set state to ExpirationValves
        if (airtime > 0){                             //if the air valve needs to open 
          digitalWrite(SV1_CONTROL, HIGH);            //open air
        } 
        if (o2time > 0){                              //if the O2 valve needs to open 
          digitalWrite(SV2_CONTROL, HIGH);            //open O2
        }
        AirStartT = millis();                         //Set time we open air valve 
        O2StartT = millis();                          //Set time we open O2 valve 
      }
      break;
      
    case ExpirationWait:
      if (millis() - ExpStartTime > breathtime){    //if expiration state has been running for desired time
        state = Inspiration;                        //set state to inspiration
        InspStartTime = millis();                   //set time we start inspiration state
      } else if (resP <=15) {                       //if the reservoir pressure is under 15 psi
        resP = respressure();                       //update reservoir pressure value
        filltime = topuptime()/2;                   //calculate new fill time
        o2fraction = 0.0127 * o2setpoint - 0.266;   //calculate fraction of O2 needed to find correct concentration 
        airfraction = 1 - o2fraction;               //calculate fraction of air needed to find correct concentration 
        o2time = o2fraction * filltime;             //calculate time for O2 valves to open
        airtime = airfraction * filltime;           //calculate time for air valves to open
        state = ExpirationValves;                   //set state to ExpirationValves
        if (airtime > 0){                           //if the air valve needs to open 
          digitalWrite(SV1_CONTROL, HIGH);          //open air
        } 
        if (o2time > 0){                            //if the O2 valve needs to open 
          digitalWrite(SV2_CONTROL, HIGH);          //open O2
        }
        AirStartT = millis();                       //set time we open aoir valve 
        O2StartT = millis();                        //set time we open O2 valve 
      }
     
      break;

    case PIDtime:
      resP = respressure();
      runPID();                                     //run PID function
      filltime = topuptime();                       //run top up time function to find fill time
      o2time = O2PID_Output * filltime;             //calculate time for O2 valves to open based on PID determined fraction
      airtime = (1 - O2PID_Output) * filltime;      //calculate time for air valves to open
      
      if (resP < 25) {
        state = ExpirationValves;                   //set state to open O2
        if (airtime > 0){                           //if the air valve needs to open 
          digitalWrite(SV1_CONTROL, HIGH);          //open air
        } 
        if (o2time > 0){                            //if the o2 valve needs to open
          digitalWrite(SV2_CONTROL, HIGH);          //open O2
        } 
        AirStartT = millis();         //Set time we open air valve 
        O2StartT = millis();          //Set time we open O2 valve 
      }else {
        state = ExpirationWait;       //set state to ExpirationWait 
      }
      break;
    
    case Timing:
      resP = respressure();         //update reservoir pressure value
      filltime = topuptime();       //calculate new fill time
      o2fraction = fraction();      //calculate fraction of O2 needed to find correct concentration 
      error = o2setpoint-o2input;   //take error between setpoint and current concentration

      if (o2fraction > 1){  //if calculated O2 fraction > 1
        o2fraction = 1;     //set O2 fraction to 1
      }
      if (o2fraction < 0){  //if calculated O2 fraction < 1
        o2fraction = 0;     //set O2 fraction to 0
      }
      airfraction = 1 - o2fraction;         //calculate fraction of air needed to find correct concentration 
      o2time = o2fraction * filltime;       //calculate time for O2 valves to open
      airtime = airfraction * filltime;     //calculate time for air valves to open
      
      if (resP < 25) {                      //if reserpoir pressure is < 25
        state = ExpirationValves;           //set state to open O2
        AirStartT = millis();
        O2StartT = millis();                //set time we start O2 valve state state
        if (airtime > 0){                   //if the air valve needs to open
          digitalWrite(SV1_CONTROL, HIGH);  //open air
        } 
        if (o2time > 0){                    //if the O2 valve needs to open
          digitalWrite(SV2_CONTROL, HIGH);  //open O2
        } 
      }else {                               //if reservoir pressure is >= 25
        state = ExpirationWait;             //set state to ExpirationWait
      }
      break;
      
    case ExpirationValves:
      resP = respressure();                       //update reservoir pressure value
    
      if (millis() - ExpStartTime > breathtime){  //if expiration state has been running for desired time
        state = Inspiration;                      //set state to inspiration
        InspStartTime = millis();                 //set time we start inspiration state
      }else if (resP >= 25){
        digitalWrite(SV1_CONTROL, LOW);           //close air
        digitalWrite(SV2_CONTROL, LOW);           //close O2
        state = ExpirationWait;                   //set state to inspiration       
      } else{
        if(airtime <= (millis() - AirStartT) && o2time <= (millis() - O2StartT)){//if air and O2 valves opened for desired time
          digitalWrite(SV1_CONTROL, LOW);           //close air
          digitalWrite(SV2_CONTROL, LOW);           //close O2
          state = ExpirationWait;                   //set state to ExpirationWait
        }else {
          if(airtime <= (millis() - AirStartT)){    //if air valve opened for desired time
          digitalWrite(SV1_CONTROL, LOW);           //close air
          }
          if(o2time <= (millis() - O2StartT)){      //if O2 valve opened for desired time
          digitalWrite(SV2_CONTROL, LOW);           //close O2
          }
        }
      }
      break;
    
    case Inspiration:
      digitalWrite(SV1_CONTROL, LOW);                 //close air
      digitalWrite(SV2_CONTROL, LOW);                 //close O2
      digitalWrite(SV4_CONTROL, LOW);                 //open expiratory valve
      analogWrite(SV3_CONTROL, 255);                  //open proportional valve (can change position)
      
      if (millis() - InspStartTime >= breathtime){    //if inspiration state has been running for desired time
        state = ExpirationNull;                       //set state to expirationNull
        ExpStartTime = millis();                      //set time we start expiration state
      }
      break;
  }
}

//-------------------Serial monitor--------------------
void report(double O2PID_Output, double filltime, double o2time,double airtime, States state) { 
  resP = respressure();                       //update reservoir pressure value
  oxygenConcentration = readOxygenSensor();   //update oxygen concentration value
  Serial.print(millis());                     //current time
  Serial.print("\t");
  Serial.print(state);                        //current state
  Serial.print("\t");
  Serial.print(resP);                         //reservoir pressure
  Serial.print("\t");
  Serial.print(O2PID_Output);                 //PID output (fraction)
  Serial.print("\t");
  Serial.print(filltime);                     //total time to fill reservoir (ms)
  Serial.print("\t");
  Serial.print(airtime);                      //calculated time air valve is left open
  Serial.print("\t");
  Serial.print(o2time);                       //calculated time o2 valve is left open
  Serial.print("\t");
  Serial.print(o2setpoint);                   //desired O2 concentration set as global variable       
  Serial.print("\t");
  Serial.println(oxygenConcentration);        //current O2 concentration                      
}

//-------------------Set Up--------------------
void setup(){
  delay(2000);                  //allows operator to start serial monitor
  pinMode(SV1_CONTROL, OUTPUT); //set air valve as an output
  pinMode(SV2_CONTROL, OUTPUT); //set O2 valve as an output
  pinMode(SV3_CONTROL, OUTPUT); //set proportional valve as an output
  pinMode(SV4_CONTROL, OUTPUT); //set expiratory valve as an output 

  pinMode(O2_SENSOR, INPUT);          //set O2 sensor as an input 
  pinMode(PRESSURE_RESERVOIR, INPUT); //set reservoir pressure sensor as an input 
  initializePID(OUTPUT_MIN, OUTPUT_MAX, SAMPLE_TIME); //run initialize() to initialize O2PID values
  Serial.begin(115200);
  Serial.println("time\tState\tP_res\tPIDout\tfillT\tairtime\tO2time\tsetpoint\tO2%");
  States state = ExpirationNull;      //initially set state to expirationnull
  ExpStartTime = millis();            //set time we start expiration state
}
  
//-------------------Loop--------------------
void loop(){
  StateMachine();                                         //run state machine 
  report(O2PID_Output, filltime, o2time, airtime, state); //run report() to print values in serial montor
} 
