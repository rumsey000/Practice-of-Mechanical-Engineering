#include <Unistep2.h>

Unistep2 stepper(4, 8, 12, 14, 4096, 2000);// IN1, IN2, IN3, IN4, 總step數, 每步的延遲(in micros)

void setup()
{
  // stepper.run();  //步進機啟動

  // if ( stepper.stepsToGo() == 0 ){ // 如果stepsToGo=0，表示步進馬達已轉完應走的step了
  //   delay(500);
  //   stepper.move(4096);    //正轉一圈
  //   //stepper.move(-4096);  //負數就是反轉，反轉一圈
  // }
}

void loop()
{
  
  stepper.run();  //步進機啟動

  if ( stepper.stepsToGo() == 0 ){ // 如果stepsToGo=0，表示步進馬達已轉完應走的step了
    delay(500);
    stepper.move(2048);    //正轉一圈
  }
}