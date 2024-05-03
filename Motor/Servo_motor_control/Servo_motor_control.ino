#include <Servo.h>   //載入函式庫，這是內建的，不用安裝

Servo leftservo;  // 建立SERVO物件
Servo rightservo;  // 建立SERVO物件


void setup() {
  Serial.begin(9600);
  leftservo.attach(10);  // 設定要將伺服馬達接到哪一個PIN腳
  rightservo.attach(9);
  
  leftservo.write(90);
  rightservo.write(0);
  delay(1000);
  //leftservo.write(0); 
  //rightservo.write(90);
  delay(1000);
  // Serial.println(leftservo.read()); 
  // delay(1000); 
  // leftservo.write(0);  
  // Serial.println(leftservo.read());  
}

void loop() {  
  Serial.print(rightservo.read()); 
  Serial.print("\t"); 
  Serial.println(leftservo.read());
    //rightservo.write(0);  //旋轉到0度，就是一般所說的歸零
  //rightservo.write(95);
  //delay(1000);
}