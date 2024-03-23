#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 // Green 
#define ENCB 3 // Yellow
#define PWM 9
#define IN2 6
#define IN1 7

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);

  int time = 0;
  while(time < 1000){
  analogWrite(PWM,200);
  time = millis();
  }
  analogWrite(PWM,0);
}

void loop() {
}



