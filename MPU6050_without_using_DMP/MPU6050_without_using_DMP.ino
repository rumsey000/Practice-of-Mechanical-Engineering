/*
The contents of this code and instructions are the intellectual property of Carbon Aeronautics. 
The text and figures in this code and instructions are licensed under a Creative Commons Attribution - Noncommercial - ShareAlike 4.0 International Public Licence. 
This license lets you remix, adapt, and build upon your work non-commercially, as long as you credit Carbon Aeronautics 
(but not in any way that suggests that we endorse you or your use of the work) and license your new creations under the identical terms.
This code and instruction is provided "As Is” without any further warranty. Neither Carbon Aeronautics or the author has any liability to any person or entity 
with respect to any loss or damage caused or declared to be caused directly or indirectly by the instructions contained in this code or by 
the software and hardware described in it. As Carbon Aeronautics has no control over the use, setup, assembly, modification or misuse of the hardware, 
software and information described in this manual, no liability shall be assumed nor accepted for any resulting damage or injury. 
By the act of copying, use, setup or assembly, the user accepts all resulting liability.

1.0  29 December 2022 -  initial release
*/

#include <Wire.h>
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;
///////////////////////////////////   SETUP   ///////////////////////////////////
void setup() {
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  //disable sleep mode
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}
///////////////////////////////////   LOOP   ///////////////////////////////////
void loop() {
  gyro_signals();
  // Serial.print("Acceleration X [g]= ");
  // Serial.print(AccX);
  // Serial.print(" Acceleration Y [g]= ");
  // Serial.print(AccY);
  // Serial.print(" Acceleration Z [g]= ");
  // Serial.print(AccZ);
  // Serial.print(" Gyroscope X [dps]= ");
  // Serial.print(RateRoll);
  // Serial.print(" Gyroscope Y [dps]= ");
  // Serial.print(RatePitch);
  // Serial.print(" Gyroscope Z [dps]= ");
  // Serial.println(RateYaw);

  Serial.print(" Roll angle [°]= ");
  Serial.print(AngleRoll);
  Serial.print(" Pitch angle [°]= ");
  Serial.println(AnglePitch);
}
///////////////////////////////////   FUNCTIONS   ///////////////////////////////////
void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  //set accelerometer full scale range in +-2g
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
  //pull accelerometer measurements from sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  //set gyroscope full scale range in +-250dps
  Wire.write(0x1B); 
  Wire.write(0x00);
  Wire.endTransmission(); 
  //pull gyroscope measurements from sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  //convert measurements to physical value
  RateRoll=(float)GyroX/131.072;
  RatePitch=(float)GyroY/131.072;
  RateYaw=(float)GyroZ/131.072;
  AccX=(float)AccXLSB/16384;
  AccY=(float)AccYLSB/16384;
  AccZ=(float)AccZLSB/16384;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}