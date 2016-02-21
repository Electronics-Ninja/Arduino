/*
  Green LED = steady on.  Power on
  Green LED = fast blink.  Warming up MQ-6 sensor
  Red LED = on 5 sec + buzzer chirp.  DHT11 sensor error.
  Red LED = steady on + buzzer = LPG concentration exceeded !
*/
#include <Wire.h>

#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"
Adafruit_7segment matrixppm = Adafruit_7segment();
Adafruit_7segment matrixtemp = Adafruit_7segment();
byte pinGreenLED = 9;
byte pinBuzzer = 3;
byte pinRedLED = 8;
byte pinNPN = 7;
byte pinMQ = A0;
boolean alarmLPG = false;
//  The lower explosive limit for LPG is 1900 ppm for butane, and
//  2000 ppm for propane.
const unsigned int LPG_threshold = 1900;
// Adjust vRef to be the true supply voltage in mV.
float vRef = 5000.0;
float RL = 20.0;  //  load resistor value in k ohms
float Ro = 10.0;  //  default value 10 k ohms.  Revised during calibration.
const float Ro_clean_air_factor = 10.0;

////////////////////////////////////////////////////////////////////////
// DHT11 humidity/temperature sensor
#include <dht11.h>
const unsigned int pinDHT = 12;
boolean errDHT11 = false;
// Uncomment whatever type you're using!
#define DHTTYPE dht11     // DHT 11 
//#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
//dht11 dht(pinDHT, DHTTYPE);
dht11 dht;

float mV = 0.0;
unsigned long samples = 0;
const int maxSamples = 500;

void setup() {
  matrixppm.begin(0x76);
  matrixtemp.begin(0x77);
  analogReference(DEFAULT);
  delay(1);
  pinMode(pinMQ, INPUT); 
  delay(1);
  pinMode(pinGreenLED, OUTPUT);
  delay(1);
  pinMode(pinRedLED, OUTPUT);
  delay(1);
  pinMode(pinBuzzer, OUTPUT);
  delay(1);
  pinMode(pinNPN, OUTPUT);
  delay(1);
  
  Serial.begin(9600);
  delay(5000);
  
  Serial.print("DHT");
  Serial.print("DHT11");
  Serial.println(" setup");  
  dht.attach(12);
  //  Get the ambient conditions (deg C & relative humidity) from DHT11
  dht.read();
  float ambRH = dht.humidity;
  float ambTemp = dht.temperature;
  int ambF = dht.fahrenheit();
  Serial.println("  ");
  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (isnan(ambTemp) || isnan(ambRH)) {
    Serial.println("Failed to read from DHT");
    Serial.println("  ");
    errDHT11 = true;
    digitalWrite(pinRedLED, HIGH);
    digitalWrite(pinBuzzer, HIGH);
    delay(100);
    digitalWrite(pinBuzzer, LOW);
    delay(5000);
    digitalWrite(pinRedLED, LOW);
  } else {
    //  DHT11 ok, .. proceed.
    Serial.print("ambTemp = ");
    Serial.print(ambTemp);
    Serial.println(" deg C");
    Serial.print("ambTemp = ");
    Serial.print(ambF);
    Serial.println(" deg F");
    Serial.print("ambRH = ");
    Serial.print(ambRH);
    Serial.println("% ");
  }
  Serial.println("  ");

  Serial.println("Calibrating MQ-6 LPG sensor in clean air..");
  Serial.println("  30 sec warmup..");
  matrixppm.print(9999, DEC);
  matrixppm.writeDisplay();
  matrixtemp.print(ambF, DEC);
  matrixtemp.writeDisplay();
  digitalWrite(pinNPN, HIGH);
  for(int i = 100; i>0; i--){
    blinkLED(pinGreenLED);
  }

  Serial.println("  30 sec warmup complete");
  
  //  take a reading..
  for(int i = 30; i>0; i--){
    blinkLED(pinGreenLED);
    mV += Get_mVfromADC(pinMQ);
    samples += 1;
  }
  digitalWrite(pinGreenLED, HIGH);
  
  mV = mV / (float) samples;
  Serial.print("  avg A");
  Serial.print(pinMQ);
  Serial.print(" for ");
  Serial.print(samples);
  Serial.print(" samples = ");
  Serial.print(mV);
  Serial.println(" mV");
  Serial.print("  Rs = ");
  Serial.println(CalcRsFromVo(mV));
  //  Conv output to Ro
  //  Ro = calibration factor for measurement in clean air.
  //  Ro = ((vRef - mV) * RL) / (mV * Ro_clean_air_factor);
  //  Hereafter, measure the sensor output, convert to Rs, and
  //  then calculate Rs/Ro using: Rs = ((Vc-Vo)*RL) / Vo
  Ro = CalcRsFromVo(mV) / Ro_clean_air_factor;
  Serial.print("  Ro = ");
  Serial.println(Ro);
  Serial.println("Sensor calibration in clean air complete");
  Serial.println("Setup complete.  Constantly monitoring for LPG..");
  Serial.println("  ");
  digitalWrite(pinNPN, HIGH);
  samples = 0;
  mV = 0.0;
}

void loop() {
  
  if (samples < maxSamples) {

    mV += Get_mVfromADC(pinMQ);
    samples += 1;
    
  } else {
    mV = mV / (float) samples;
    Serial.print("  avg A");
    Serial.print(pinMQ);
    Serial.print(" for ");
    Serial.print(samples);
    Serial.print(" samples = ");
    Serial.print(mV);
    Serial.println(" mV");
    //  Conv output to Ro
    //  Calculate Rs from the measured sensor voltage output
    //  using: Rs = (Vc-Vo)* (RL / Vo)
    //float Rs = (vRef - mV) * (RL / mV);
    float Rs = CalcRsFromVo(mV);
    Serial.print("Rs = ");
    Serial.println(Rs);
    //  Calculate Rs/Ro
    float RsRo_atAmb = Rs / Ro;
    //  Get the ambient conditions (deg C & relative humidity) from DHT11
    dht.read();
    float ambRH = dht.humidity;
    float ambTemp = dht.temperature;
    int ambF = dht.fahrenheit();
    // check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(ambTemp) || isnan(ambRH)) {
      Serial.println("Failed to read from DHT");
      errDHT11 = true;
      digitalWrite(pinNPN, LOW);
    } else {
      //  DHT11 ok, .. proceed.
      errDHT11 = false;
      Serial.print("RsRo @ ambient = ");
      Serial.print(RsRo_atAmb);
      Serial.print("; ambTemp = ");
      Serial.print(ambTemp);
      Serial.print("; ambF = ");
      Serial.print(ambF);
      Serial.print("; ambRH = ");
      Serial.print(ambRH);
      Serial.println("% ");
      float measured_RsRo_at_20C65RH = RsRoAtAmbientTo20C65RH(RsRo_atAmb, ambTemp, ambRH);
      Serial.print("measured_RsRo_at_20C65RH = ");
      Serial.println(measured_RsRo_at_20C65RH);
      //  Calc the measured Rs/Ro (corrected to std 20C and 65% RH) in LPG
      //  concentration (ppm).
      Serial.print("LPG ppm = ");
      unsigned int LPG_ppm = GetLpgPpmForRatioRsRo(measured_RsRo_at_20C65RH);
      Serial.println(LPG_ppm);
      Serial.println("  ");
      digitalWrite(pinNPN, LOW);
      matrixppm.print(LPG_ppm, DEC);
      matrixppm.writeDisplay(); 
      matrixtemp.print(ambF, DEC);
      matrixtemp.writeDisplay(); 
            
      if (LPG_ppm > LPG_threshold) {
        alarmLPG = true;
      } else {
        alarmLPG = false;
      }
    }
    samples = 0;
    mV = 0.0;
  }
  
  if (alarmLPG == true) {
    digitalWrite(pinRedLED, HIGH);
    digitalWrite(pinBuzzer, HIGH);
  }
  
  if (errDHT11 == true) {
    digitalWrite(pinRedLED, HIGH);
    digitalWrite(pinBuzzer, HIGH);
    delay(100);
    digitalWrite(pinBuzzer, LOW);
    delay(5000);
    digitalWrite(pinRedLED, LOW);
  }
   
}

float RsRoAtAmbientTo20C65RH(float RsRo_atAmb, float ambTemp, float ambRH) {
  //  Using the datasheet for MQ-6 sensor, derive Rs/Ro values 
  //  from - 10 to 50 C and 33, 65, and 85 % relative humidity.
  //  For the measured Rs/Ro, use linear interpolation to calculate the
  //  standard Rs/Ro values for the measured ambient temperature and RH.
  //  Next, calculate a correction factor from the standard Rs/Ro at ambient
  //  temp and RH relative to standard Rs/Ro at 20C and 65 RH.  
  //  Apply this correction factor to the measured Rs/Ro value and return the
  //  corrected value.  This corrected value may then be used against the Rs/Ro
  //  Rs/Ro vs LPG concentration (ppm) chart to estimate the concentration of PPG.
  
  //  Calc RsRo values at ambTemp & 33% RH, 65% and 85% RH
  float RsRo_at_ambTemp_33RH = -0.00000321 * pow(ambTemp, 3) + 0.000315 * pow(ambTemp, 2) - 0.0129 * ambTemp + 1.1568;
  float RsRo_at_ambTemp_85RH = -0.00000358 * pow(ambTemp, 3) + 0.000298 * pow(ambTemp, 2) - 0.0102 * ambTemp + 0.9642;
   //float RsRo_at_65RH = ((65.0-33.0)/(85.0-65.0));
  float RsRo_at_ambTemp_65RH = ((65.0-33.0)/(85.0-33.0)*(RsRo_at_ambTemp_85RH-RsRo_at_ambTemp_33RH)+RsRo_at_ambTemp_33RH)*1.102;
  //  Linear interpolate to get the RsRo at the ambient RH value (ambRH).
  float RsRo_at_ambTemp_ambRH;
  if (ambRH < 65.0) {
    RsRo_at_ambTemp_ambRH = (ambRH - 33.0)/(65.0 - 33.0)*(RsRo_at_ambTemp_65RH - RsRo_at_ambTemp_33RH) + RsRo_at_ambTemp_33RH;  
  } else {
    // ambRH > 65.0
    RsRo_at_ambTemp_ambRH = (ambRH - 65.0)/(85.0 - 65.0)*(RsRo_at_ambTemp_85RH - RsRo_at_ambTemp_65RH) + RsRo_at_ambTemp_65RH;
  }
  //  Calc the correction factor to bring RsRo at ambient temp & RH to 20 C and 65% RH.
  const float refRsRo_at_20C65RH = 1.00;
  float RsRoCorrPct = 1 + (refRsRo_at_20C65RH - RsRo_at_ambTemp_ambRH)/refRsRo_at_20C65RH;
  //  Calculate what the measured RsRo at ambient conditions would be corrected to the
  //  conditions for 20 C and 65% RH.
  float measured_RsRo_at_20C65RH = RsRoCorrPct * RsRo_atAmb;
  return measured_RsRo_at_20C65RH;
}

float CalcRsFromVo(float Vo) {
  //  Vo = sensor output voltage in mV.
  //  VRef = supply voltage, 5000 mV
  //  RL = load resistor in k ohms
  //  The equation Rs = (Vc - Vo)*(RL/Vo)
  //  is derived from the voltage divider
  //  principle:  Vo = RL * Vc (Rs + RL)
  //
  //  Note.  Alternatively you could calc
  //         Rs from ADC value using
  //         Rs = RL * (1024 - ADC) / ADC
  float Rs = (vRef - Vo) * (RL / Vo);  
  return Rs;
}

unsigned int GetLpgPpmForRatioRsRo(float RsRo_ratio) {
  //  If you extract the data points from the LPG concentration
  //  versus Rs/Ro chart in the datasheet, plot the points,
  //  fit a polynomial curve to the points, you come up with the equation
  //  for the curve of:  Rs/Ro = 18.446 * (LPG ppm) ^ -0.421
  //  This equation is valid for ambient conditions of 20 C and 65% RH.
  //  Solving for the concentration of LPG you get:
  //    LPG ppm = [(Rs/Ro)/18.446]^(1/-0.421)
  float ppm;
  ppm = pow((RsRo_ratio/18.446), (1/-0.421));
  return (unsigned int) ppm;
}

float Get_mVfromADC(byte AnalogPin) {
    // read the value from the sensor:
    int ADCval = analogRead(AnalogPin);  
    // It takes about 100 microseconds (0.0001 s) to read an analog input
    delay(1);
    //  Voltage at pin in milliVolts = (reading from ADC) * (5000/1024) 
    float mV = ADCval * (vRef / 1024.0);
    return mV;
}

void blinkLED(byte ledPIN){
  //  consumes 300 ms.
  for(int i = 5; i>0; i--){
    digitalWrite(ledPIN, HIGH);
    delay(30);
    digitalWrite(ledPIN, LOW);
    delay(30);
  }    
}
