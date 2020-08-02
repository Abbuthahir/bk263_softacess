#include "DFRobot_BME680_I2C.h"
#include "Wire.h"
#include "SPI.h"

/*char inchar; use only send alert as message to mobile

/*use an accurate altitude to calibrate sea level air pressure*/
#define CALIBRATE_PRESSURE

DFRobot_BME680_I2C bme(0x77);  //0x77 I2C address

float seaLevel;
void setup()
{
  pinMode(16, OUTPUT);
  /*digitalWrite(16, LOW);*/
  Serial.begin(115200);
  while(!Serial);
  delay(1000);
  Serial.println();
  Serial.print(bme.begin());
  #ifdef CALIBRATE_PRESSURE
  bme.startConvert();
  delay(1000);
  bme.update();
  /*You can use an accurate altitude to calibrate sea level air pressure.
   *And then use this calibrated sea level pressure as a reference to obtain the calibrated altitude.
   *In this case,106.0m is kovilpatti accurate altitude.
   */
  seaLevel = bme.readSeaLevel(106.0);
  //Serial.print("seaLevel :");
  //Serial.println(seaLevel);
  #endif
}

void loop()
{
  float T,P,H,IAQ,A,PIAQ,CA;
  bme.startConvert();
  delay(3000);
  bme.update();
  T=bme.readTemperature()/100;
  P=bme.readPressure()/101325;
  H=bme.readHumidity()/1000;
  IAQ=bme.readGasResistance();
  PIAQ=(IAQ/200000)*100;
  A=bme.readAltitude();
  CA=bme.readCalibratedAltitude(seaLevel);

  if(PIAQ<60){
    Serial.println("1. System Alert AIR Quality Afftected Seams Fire Caught Inside the Warehouse");
  }
  
  if(T>36){
    digitalWrite(16, LOW);
    delay(1000);
    digitalWrite(16, HIGH);
    Serial.println("2. System Alert temperature reached the threshold level");
  }
  else{digitalWrite(16, LOW);}

 
  Serial.println();
  Serial.print("Temperature(C) :");
  Serial.println(T);
  Serial.print("Pressure(ATM) :");
  Serial.println(P);
  Serial.print("Humidity(%RH) :");
  Serial.println(H);
  Serial.print("Indoor Air Quality(%) :");
  Serial.println(PIAQ);
  Serial.print("Altitude(m) :");
  Serial.println(A);
  #ifdef CALIBRATE_PRESSURE
  Serial.print("Calibrated Altitude(m) :");
  Serial.println(CA);
  #endif
}
