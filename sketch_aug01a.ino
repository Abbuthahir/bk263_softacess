#include "DFRobot_BME680_I2C.h"
#include "Wire.h"
#include "SPI.h"

#define CALIBRATE_PRESSURE

DFRobot_BME680_I2C bme(0x77);  

float seaLevel;
void setup()
{
Serial.begin(115200);
while(!Serial);
delay(000);
Serial.println();
Serial.print(bme.begin());
#ifdef CALIBRATE_PRESSURE
bme.startConvert();
delay(1000);
bme.update();

seaLevel = bme.readSeaLevel(106.0);

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
