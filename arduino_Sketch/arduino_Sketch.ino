#include <PID_v1.h>
#include <Adafruit_MAX31865.h>

Adafruit_MAX31865 max = Adafruit_MAX31865(8, 11, 12, 13);
#define RREF      430.0
#define RNOMINAL  100.0

double fast = 90;
double fastinc = 5;

double tempread;
double Input, Output, Setpoint;
double tempset = 21;
double Kp = 250;
double Ki = 0;
double Kd = 0;
int stirmotor = 5;
int chill = 3;
int blueled = 9;
int redled = 10;
unsigned long blue_previousMillis = -240000;
float blue_interval = 300000; 
boolean blueflag = false;
boolean checktemp= true;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //Starting the PID, Specify the links and initial tuning parameters

void setup() {
  Serial.begin(115200);
  max.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
   //Setting PID parameters////////////////////////////////////////////////////////////////////////////////////////
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(80, 255);
  Serial.println("Test start");
  Serial.println("millis,temp,output,tempset");
}


void loop() {


unsigned long blue_currentMillis = millis();
  if (blue_currentMillis - blue_previousMillis >= blue_interval) {
    blue_previousMillis = blue_currentMillis;
    if (blueflag) {
      blueflag=false;
    }
    else {
      blueflag=true;
    }
  
  }

  if (blueflag) {
    analogWrite(blueled, 255);
  }
  else {
    analogWrite(blueled, 0);
  }
  
  analogWrite(redled, 100);
  analogWrite(stirmotor, fast);
  if(fast<150){
    fast = fast + fastinc;
  }
  uint16_t rtd = max.readRTD();
  
  float ratio = rtd;
  ratio /= 32768;
  tempread=max.temperature(RNOMINAL, RREF);
  Input = -1 * tempread;
  Setpoint = -1 * tempset;
  myPID.Compute();
  analogWrite(chill,Output);


  // Check and print any faults
  uint8_t fault = max.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    max.clearFault();
  }

  //Serial.print("Speed is: ");
  //Serial.println(fast);
  //Serial.print("RTD value: "); Serial.println(rtd);
  //Serial.print("Ratio = "); Serial.println(ratio,8);
  //Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  //Serial.print("Temperature = "); Serial.println(max.temperature(RNOMINAL, RREF));
  //Serial.print("Output level: "); Serial.println(Output);
  //Serial.println();

  Serial.print(millis());
  Serial.print(",");
  Serial.print(tempread);
  Serial.print(",");
  Serial.println(Output);
  Serial.print(",");
  Serial.println(tempset);
  delay(843);
}
