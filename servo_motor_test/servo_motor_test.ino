#include <Servo.h>   //載入函式庫，這是內建的，不用安裝

Servo leftservo;  // 建立SERVO物件
Servo rightservo;  // 建立SERVO物件


void setup() {
  leftservo.attach(10);  // 設定要將伺服馬達接到哪一個PIN腳
  rightservo.attach(9);
}

void loop() {   
  leftservo.write(0);  //旋轉到0度，就是一般所說的歸零
  rightservo.write(0);  //旋轉到0度，就是一般所說的歸零
  delay(1000);
  leftservo.write(90); //旋轉到90度
  rightservo.write(90);
  delay(1000);
  leftservo.write(180); //旋轉到180度
  rightservo.write(180);
  delay(1000);
}