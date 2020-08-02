#include<Wire.h>
#include<DFRobot_LIS2DH12.h>
DFRobot_LIS2DH12 LIS; 
void setup(){
Wire.begin();
Serial.begin(115200);
while(!Serial);
delay(100);
while(LIS.init(LIS2DH12_RANGE_16GA) == -1){  //Equipment connection exception or I2C address error
Serial.println("No I2C devices found");
delay(1000);}}

void loop(){
  acceleration();}
void acceleration(void)
{
  int16_t x, y, z;
delay(100);
LIS.readXYZ(x, y, z);
LIS.mgScale(x, y, z);
Serial.print(x);            
Serial.print('\t');        
Serial.print(y);        
Serial.print('\t');      
Serial.println(z);        
}
