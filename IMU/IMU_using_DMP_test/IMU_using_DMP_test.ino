#include <Servo.h>// Servo Motor Library   
// IMU Library
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
Servo myservo;
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
// #define OUTPUT_READABLE_REALACCEL
// #define OUTPUT_READABLE_WORLDACCEL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED 13
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
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
void setup() {
  IMU_Initialize();
  myservo.attach(8);
  myservo.write(90); 
  pinMode(LED, OUTPUT);
}
void loop() {
  IMU_Read();
  //Servo Motor
  myservo.write(90+yaw); 
  // digitalWrite(LED, HIGH);  // turn the LED on (HIGH is the voltage level)
  // delay(100);                      // wait for a second
  // digitalWrite(LED, LOW);   // turn the LED off by making the voltage LOW
  // delay(100); 
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
  pinMode(INTERRUPT_PIN, INPUT);
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
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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