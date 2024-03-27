#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define BUTTON_PIN 3  // interrupt only 2 3 available
#define ENCA 2 // Green interrupt only 2 3 available
#define ENCB 5 // Yellow
#define PWM 9
#define IN2 6
#define IN1 7
#define LED 13
volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;

float gearRatio = 45.0; 
float encoderResolution = 11.0;
float target_angle = 90.0;
float current_angle = 0.0;


int buttonState = 0;   
bool run_motor = false; // false == 0 ,true = !false

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP); //設定按鈕的接腳為輸入並附帶上拉電阻
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(LED,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN),change_motor_state,FALLING);
}

void loop() {
  buttonState = digitalRead(BUTTON_PIN);  //讀取按鍵的狀態
  
  if(run_motor == true){ 
    digitalWrite(LED,HIGH);      
    // 運轉馬達到特定角度

    // set target position
    int target = gearRatio * encoderResolution * target_angle / 360.0;

    // PID constants
    float kp = 1.45;
    float kd = 0.0;
    float ki = 0.0;

    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Read the position in an atomic block to avoid a potential
    // misread if the interrupt coincides with this code running
    // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
    int pos = 0; 
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      pos = posi;
    }
      // error
    int e = pos - target;
    // derivative
    float dedt = (e-eprev)/(deltaT);
    // integral
    eintegral = eintegral + e*deltaT;
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
    // motor power
    float pwr = fabs(u);
    if( pwr > 255 ){
      pwr = 255;
    }
    // motor direction
    int dir = 1;
    if(u<0){
      dir = -1;
    }
    // signal the motor
    setMotor(dir,pwr,PWM,IN1,IN2);
    // store previous error
    eprev = e;
    current_angle = pos * 360.0 / gearRatio / encoderResolution;
    Serial.print(target);
    Serial.print(",");
    Serial.print(pos);
    Serial.print(",");
    Serial.print(target_angle);
    Serial.print(",");
    Serial.print(current_angle);
    Serial.println(",");   
  }else{                        //如果按鍵是未按下
    digitalWrite(LED,LOW); 
    setMotor(0,0,PWM,IN1,IN2);  // 停止馬達運作
  }
  Serial.print("BOTTON:");
  Serial.print(digitalRead(BUTTON_PIN));
  Serial.print("run_motor:");
  Serial.print(run_motor);
  Serial.print("LED:");
  Serial.println(digitalRead(LED));
}
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}
void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
void change_motor_state(){
  run_motor = !run_motor;
}