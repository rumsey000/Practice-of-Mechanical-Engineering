// Library
#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <Servo.h> // For servo motor control
// IO PIN
#define BUTTON 3  // interrupt only 2 3 available
#define ENCA 2 // Green interrupt only 2 3 available
#define ENCB 5 // Yellow
#define PWM 11
#define IN2 6
#define IN1 7
#define LED 13
// PID Parameters
volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
// Encoder Parameters
float gear_ratio = 45.0; 
float encoder_resolution = 11.0;
float current_angle = 0.0;
// Botton Parameter
int led_state = LOW;      
int button_state;          
int last_button_state = HIGH;  
unsigned long last_debounce_time = 0; 
unsigned long debounce_delay = 50;   
// Motor State
int state = 0;
// Servo motor declare
Servo leftservo;
Servo rightservo;
////////////////////        SETUP        ////////////////////

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON, INPUT_PULLUP);; //set as pull-up resistor
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(LED,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  leftservo.attach(9);
  rightservo.attach(10);
}
////////////////////        LOOP        ////////////////////
void loop() {
  if (state==0){
    // Read and debounce botton 
    leftservo.write(0);
    rightservo.write(90);
    ReadAndDebounceBotton();
  }
  if (state==1){
    delay(2000);
    state++;
  }
  if(state == 2){ 
    // Forward rotation
    PIDControlMotor(150.0,0.86,0.0,0.0);
    leftservo.write(90);
    rightservo.write(0);
    // ReadAndDebounceBotton 
    ReadAndDebounceBotton();   
  }
  if (state==3){
    // Reverse servo motor
    leftservo.write(0);
    rightservo.write(90);  
    // ReadAndDebounceBotton 
    ReadAndDebounceBotton();   
  }
  if (state==4){
    // Reverse dc motor
    PIDControlMotor(0.0,0.868,0.0,0.0);
    // ReadAndDebounceBotton 
    ReadAndDebounceBotton();
  }
  if (state==5){
    // Stop motor
    setMotor(0,0,PWM,IN1,IN2);
    delay(500);
    state = 0;
  }
  Serial.println(state);
}
////////////////////        FUNCTION        ////////////////////
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
void ReadAndDebounceBotton(){
  int reading = digitalRead(BUTTON);
  if (reading != last_button_state) {
    last_debounce_time = millis();
  }
  if ((millis() - last_debounce_time) > debounce_delay) {
    if (reading != button_state) {
      button_state = reading;
      if (button_state == LOW) {
        led_state = !led_state;
        state++;
      }
    }
  }
  digitalWrite(LED,led_state);
  last_button_state = reading;
}
void PIDControlMotor(float target_angle,float kp,float ki,float kd){
  int target = gear_ratio * encoder_resolution * target_angle / 360.0;
  // Time difference
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
  if( pwr > 190 ){ //   256*(12/15.8) = 194.4 
    pwr = 190;
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
  current_angle = pos * 360.0 / gear_ratio / encoder_resolution;
  Serial.print(target);
  Serial.print(",");
  Serial.print(pos);
  Serial.print(",");
  Serial.print(target_angle);
  Serial.print(",");
  Serial.print(current_angle);
  Serial.println(",");
  Serial.print(pwm);
  Serial.println(",");
}
