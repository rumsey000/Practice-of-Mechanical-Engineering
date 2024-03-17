
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps612.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int calib_count = 1;
double cax =0;  double cay =0;  double caz =0;
double cgx =0;  double cgy =0;  double cgz =0;
bool run_calib = true;
double offset_aa_x = -23.19;
double offset_aa_y = -4.63;
double offset_aa_z = 19.49;
double offset_gy_x = 0.65;
double offset_gy_y = 0.72;
double offset_gy_z = 0.62;
double offset_yaw = -1.69;
double offset_pitch = -0.01;
double offset_roll = -0.06;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
   Serial.begin(38400);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

   // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
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

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees (-180° ~ 180°)
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGyro(&gy, fifoBuffer);
    Serial.print("yaw : ");
    Serial.print(ypr[0] * 180 / M_PI + offset_yaw);
    Serial.print("[°]  ");
    Serial.print("pitch : ");
    Serial.print(ypr[1] * 180 / M_PI + offset_pitch);
    Serial.print("[°]  ");
    Serial.print("roll : ");
    Serial.print(ypr[2] * 180 / M_PI + offset_roll);
    Serial.print("[°]  ");
    
    // Serial.print("Accel X : ");
    // Serial.print((aa.x + offset_aa_x) / 16384.0 );
    // Serial.print("[g]  ");
    // Serial.print("Accel Y : ");
    // Serial.print((aa.y + offset_aa_y) / 16384.0 );
    // Serial.print("[g]  ");
    // Serial.print("Accel Z : ");
    // Serial.print((aa.z + offset_aa_z) / 16384.0 );
    // Serial.print("[g]  ");
    // Serial.print("Gyro X : ");
    // Serial.print((gy.x + offset_gy_x) / 65536.0 *2000);
    // Serial.print("[dps]  ");
    // Serial.print("Gyro Y : ");
    // Serial.print((gy.y + offset_gy_y) / 65536.0 *2000);
    // Serial.print("[dps]  ");
    // Serial.print("Gyro Z : ");
    // Serial.print((gy.z + offset_gy_z) / 65536.0 *2000);
    // Serial.print("[dps]  ");
    Serial.println();
#endif
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }

  /*Check FullScaleRange Part
    //Serial.print("GyroRange:");
    //Serial.print(mpu.getFullScaleGyroRange());
    // 0 = +/- 250 degrees/sec
    // 1 = +/- 500 degrees/sec
    // 2 = +/- 1000 degrees/sec *
    // 3 = +/- 2000 degrees/sec
    //Serial.print("\tAccelRange:");
    //Serial.print(mpu.getFullScaleAccelRange());
    // 0 = +/- 2g *
    // 1 = +/- 4g
    // 2 = +/- 8g
    // 3 = +/- 16g
  */

  //Calibration Part
  if(calib_count < 20000){
    cax += aa.x;
    cay += aa.y;
    caz += aa.z;
    cgx += gy.x;
    cgy += gy.y;
    cgz += gy.z;

    // Serial.print("aa.z: ");
    // Serial.print(aa.z);
    // Serial.print("\tcaz : ");
    // Serial.print(caz);
    // Serial.print("\tcalib_count : ");
    // Serial.println(calib_count);
    
    // Serial Plotter
    // Serial.print("accel_offset_x:");
    // Serial.println( 0 - cax/calib_count );
    // Serial.print("accel_offset_y:");
    // Serial.println( 0 - cay/calib_count );
    // Serial.print("accel_offset_z:");
    // Serial.println( 16384 - caz/calib_count );

    // Serial.print("gyro_offset_x:");
    // Serial.println( 0 - cgx/calib_count );
    // Serial.print("gyro_offset_y:");
    // Serial.println( 0 - cgy/calib_count );
    // Serial.print("gyro_offset_z:");
    // Serial.println(0 - cgz/calib_count );

    // Serial.print("yaw:");
    // Serial.println(0 - ypr[0] * 180 / M_PI);
    // Serial.print("pitch:");
    // Serial.println(0 - ypr[1] * 180 / M_PI);
    // Serial.print("roll:");
    // Serial.println(0 - ypr[2] * 180 / M_PI);

    // Serial Monitor
    // Serial.print("accel_offset_xyz:");
    // Serial.println( 0 - cax/calib_count );
    // Serial.print("   ");
    // Serial.print( 0 - cay/calib_count );
    // Serial.print("   ");
    // Serial.println( 16384 - caz/calib_count );
    // Serial.print("gyro_offset_xyz: ");
    // Serial.print( 0 - cgx/calib_count );
    // Serial.print("   ");
    // Serial.print( 0 - cgy/calib_count );
    // Serial.print("   ");
    // Serial.println(0 - cgz/calib_count );
    calib_count = calib_count + 1;
  }
  else{
      run_calib = false;
  }
  if(run_calib = true){}
  else{
    //stop add calib_count 
  }
  //
}
