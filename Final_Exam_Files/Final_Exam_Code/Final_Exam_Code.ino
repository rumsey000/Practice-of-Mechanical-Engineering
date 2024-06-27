// Library
#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <Servo.h> // For servo motor control
#include "I2Cdev.h"// For I2C communicate
#include "MPU6050_6Axis_MotionApps612.h"//For IMU
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
// Declaration
Servo leftservo;// Servo motor 
Servo rightservo;// Servo motor 
Servo turnservo;// Servo motor 
Servo liftservo;// Servo motor 
Servo right_brake_servo;  
Servo left_brake_servo; 
MPU6050 mpu;// IMU
//BOT &LED PIN
#define BUTTON 6  // interrupt only 2 3 available
#define LED 13
// measure distance parameter
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
#define OUTPUT_READABLE_YAWPITCHROLL
// #define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
//time interval
#define interval_1 500
#define interval_2 1500
#define interval_3 3800
#define interval_4 2000
#define interval_5 5000 
// Botton Parameter
int led_state = LOW;      
int button_state;          
int last_button_state = HIGH;  
unsigned long last_debounce_time = 0; 
unsigned long debounce_delay = 50;   
// Motor State
int state = 0;
int i = 0;
int pos_left = 145;
int pos_right = 35;
int pos_lift = 10;
int pos_turn = 90;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double yaw;
// time_interval vars
unsigned long current_time = 0;
unsigned long start_time = 0;
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
////////////////////        SETUP        ////////////////////

void setup() {
  pinMode(BUTTON, INPUT_PULLUP);; //set as pull-up resistor
  pinMode(LED,OUTPUT);
  IMU_Initialize();
  right_brake_servo.attach(4);
  left_brake_servo.attach(5);
  liftservo.attach(8);
  leftservo.attach(9);
  rightservo.attach(10);
  turnservo.attach(11);
  right_brake_servo.write(90);
  left_brake_servo.write(90);
  pinMode(leftphoto, INPUT);
  pinMode(rightphoto, INPUT);
  attachInterrupt(digitalPinToInterrupt(rightphoto),rightcount,RISING);
  attachInterrupt(digitalPinToInterrupt(leftphoto),leftcount,RISING);
}
////////////////////        LOOP        ////////////////////
void loop() {
  Serial.print("state:");
  Serial.println(state);
  if (state==0){
    liftservo.write(10);
    leftservo.write(145);
    rightservo.write(35);
    turnservo.write(90);
    ReadAndDebounceBotton();
  }
  if (state==1){
    led_state = !led_state;// LED dark->light
    digitalWrite(LED,led_state); //LED light
    for(start_time = millis() ;  current_time < start_time + interval_1 ; current_time = millis()){
    }    
    state++;
  }
  if(state == 2){ 
    brake();
    open_sails();
    cancel_brake();
    state++;
  }
  if (state == 3){
    turn_sail(160);
    for(start_time = millis() ;  current_time < start_time + interval_2 ; current_time = millis()){
      brake();
    }
    cancel_brake();
    for(start_time = millis() ;  current_time < start_time + interval_3 ; current_time = millis()){
      turn_sail_with_IMU(160);
    }
    for(start_time = millis() ;  current_time < start_time + interval_4 ; current_time = millis()){
      brake();
    }
    cancel_brake();
    for(start_time = millis() ;  current_time < start_time + interval_5 ; current_time = millis()){
      turn_sail_with_IMU(15);
    }  
    state++;
  }
  if (state == 4){
    ReadAndDebounceBotton();
    Serial.println(pos_turn);
  }
  if (state == 5){
    led_state = !led_state;// light->dark
    digitalWrite(LED,led_state); //dark
    close_turn_sail();
    close_main_sail();
    close_side_sails();
    state = 0; 
  }
}
////////////////////        FUNCTION        ////////////////////
void ReadAndDebounceBotton(){
  int reading = digitalRead(BUTTON);
  if (reading != last_button_state) {
    last_debounce_time = millis();
  }
  if ((millis() - last_debounce_time) > debounce_delay) {
    if (reading != button_state) {
      button_state = reading;
      if (button_state == LOW) {
        state++;
      }
    }
  }
  last_button_state = reading;
}
void open_sails(){
  while((pos_lift < 175) && ( pos_left > 45 && pos_right < 135 )){
      liftservo.write(pos_lift); 
      leftservo.write(pos_left); 
      rightservo.write(pos_right);
      delay(20);
      if(pos_lift < 175){
        pos_lift += 2;
      }
      Serial.print((pos_lift < 175) && ( pos_left > 45 && pos_right < 135 ));
      Serial.print("\t");
      Serial.println(pos_lift);
      if(pos_left > 45 && pos_right < 135){
        pos_left -= 1;
        pos_right += 1;
      }
    }
}
void IMU_Initialize(){
    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    Wire.setWireTimeout(3000, true);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  // initialize serial communication
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  //pinMode(INTERRUPT_PIN, INPUT);
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // wait for ready
  // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  // while (!Serial.available());                 // wait for data
  // while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(15);
    mpu.CalibrateGyro(15);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    Serial.println(mpuInterrupt);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  //Check FullScaleRange 
    Serial.print("GyroRange:");
    Serial.print(mpu.getFullScaleGyroRange());
    // 0 = +/- 250 degrees/sec
    // 1 = +/- 500 degrees/sec
    // 2 = +/- 1000 degrees/sec
    // 3 = +/- 2000 degrees/sec *
    Serial.print("\tAccelRange:");
    Serial.print(mpu.getFullScaleAccelRange());
    // 0 = +/- 2g *
    // 1 = +/- 4g
    // 2 = +/- 8g
    // 3 = +/- 16g
}
void IMU_Read(){
// if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    /*
      mpu.dmpGetAccel(&aa, fifoBuffer);
      Serial.print("\tRaw Accl XYZ\t");
      Serial.print(aa.x);
      Serial.print("\t");
      Serial.print(aa.y);
      Serial.print("\t");
      Serial.print(aa.z);
      mpu.dmpGetGyro(&gy, fifoBuffer);
      Serial.print("\tRaw Gyro XYZ\t");
      Serial.print(gy.x);
      Serial.print("\t");
      Serial.print(gy.y);
      Serial.print("\t");
      Serial.print(gy.z);
    */
    Serial.println();
    yaw = ypr[0] *180 / M_PI;
    #endif
  }
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
float Distance(){
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
  return(x);
}
void brake(){
  right_brake_servo.write(80);
  left_brake_servo.write(99);
}
void cancel_brake(){
  right_brake_servo.write(90);
  left_brake_servo.write(90);
}
void turn_sail(int turn_angle){
  while(pos_turn < turn_angle){
    turnservo.write(pos_turn); delay(10);
    pos_turn += 1;
  }
}
void turn_sail_with_IMU(int turn_angle){
  IMU_Read();
  pos_turn = turn_angle + yaw;
  if(pos_turn < 0){
    pos_turn = 0;
  }
  if(pos_turn > 180){
     pos_turn =  180;
  }
  turnservo.write(pos_turn);
  Serial.println(pos_turn);
}
void close_turn_sail(){
  if(pos_turn > 90){
    while(pos_turn > 90){
    turnservo.write(pos_turn); delay(20);
    pos_turn -= 1;
    Serial.print("pos_turn:");
    Serial.println(pos_turn);
    }
  }
  else{
    while(pos_turn < 90){
    turnservo.write(pos_turn); delay(20);
    pos_turn += 1;
    Serial.print("pos_turn:");
    Serial.println(pos_turn);
    }
  }
}
void close_main_sail(){
  while(pos_lift > 10){
    liftservo.write(pos_lift); delay(20);
    pos_lift -= 1;
  }
}
void close_side_sails(){
  while(pos_left < 145 && pos_right > 35){
    leftservo.write(pos_left); delay(5);
    rightservo.write(pos_right); delay(5);
    pos_left += 1;
    pos_right -= 1;
  }   
}