#include <util/atomic.h>
#define leftphoto 2 
#define rightphoto 3
volatile long leftcounti = 0;
volatile long rightcounti = 0;
float leftphi = 0;
float rightphi = 0;
float d_leftphi = 0;
float d_rightphi = 0;
float last_leftphi = 0;
float last_rightphi = 0;
float leftdisplacement = 0;
float rightdisplacement = 0;
float wheelradius = 1.5;
float wheeldistance = 14.88;
long  leftsensorRead = 0;
long  rightsensorRead = 0;
double x = 0,y = 0,theta = 0;
unsigned long right_time = 0;
unsigned long last_right_time = 0;
unsigned long left_time = 0;
unsigned long last_left_time = 0;
unsigned long time = 0;
void setup() {
  Serial.begin(115200);
  pinMode(leftphoto, INPUT);
  pinMode(rightphoto, INPUT);
  attachInterrupt(digitalPinToInterrupt(rightphoto),rightcount,RISING);
  attachInterrupt(digitalPinToInterrupt(leftphoto),leftcount,RISING);
}

void loop() {
  // wheel distance
  leftsensorRead = digitalRead(leftphoto); 
  rightsensorRead = digitalRead(rightphoto);
  Serial.print("rightsensorRead:");
  Serial.print(rightsensorRead);
  Serial.print("\tleftsensorRead:");
  Serial.print(leftsensorRead);
  long left_count = 0;
  long right_count = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    right_count = rightcounti;
    left_count = leftcounti;
  }
  Serial.print("\trightcount:");
  Serial.print(rightcounti);
  Serial.print("\tleftcount:");
  Serial.print(leftcounti);
  if (millis() - time >= 1){ 
    rightphi = right_count * 2.0 * PI / 24.0;
    leftphi = left_count * 2.0 * PI / 24.0;
    d_rightphi = rightphi - last_rightphi;
    d_leftphi =  leftphi - last_leftphi;
    theta += (wheelradius*d_rightphi/ (wheeldistance) ) - (wheelradius * d_leftphi/ (wheeldistance) );
    x += cos(theta) * (wheelradius*d_rightphi/2.0 + wheelradius*d_leftphi/2.0);
    y += sin(theta) * (wheelradius*d_rightphi/2.0 + wheelradius*d_leftphi/2.0);
    time = millis();

    last_rightphi = rightphi;
    last_leftphi =  leftphi;
  }

  Serial.print("\t x:");
  Serial.print(x);
  Serial.print("\t y:");
  Serial.print(y);
  Serial.print("\t theta:");
  Serial.println(theta);
}
void rightcount(){
  right_time = millis();
  if (right_time - last_right_time >= 50) {
    rightcounti++;
  }
  last_right_time = right_time;
}
void leftcount(){
  left_time = millis();
  if (left_time - last_left_time >= 50) {
    leftcounti++;
  }
  last_left_time = left_time;
}
