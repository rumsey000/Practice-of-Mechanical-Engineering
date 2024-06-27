#include <Servo.h>   //載入函式庫，這是內建的，不用安裝
Servo right_brake_servo;  
Servo left_brake_servo; 
void setup() {
  Serial.begin(115200);
  right_brake_servo.attach(4);
  left_brake_servo.attach(5);   
  right_brake_servo.write(90);
  left_brake_servo.write(90); 
  delay(1000);
}
void loop() {  
  // right_brake_servo.write(80);
  // left_brake_servo.write(102); 
}