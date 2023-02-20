#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "GyverFilters.h"

#define pot0 A0         //пины потенциопетров(1-4)
#define pot1 A1
#define pot2 A2
#define pot3 A3
#define waitTime 10000

const float kp = 0.9;       //коэффицент для фильтра потенциометров (0.01-1.0)
const float kg = 0.7;       //коэффицент для гироскопа (0.01-1.0)
const int step = 10;
const int NUM_READ = 3;

MPU6050 mpu;

GFilterRA analog1;
GFilterRA analog2;
GFilterRA analog3;
GFilterRA analog4;

GFilterRA gyro1;
GFilterRA gyro2;
GFilterRA gyro3;
GFilterRA gyro4;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


int amin0, amin1, amin2, amin3;
int amax0, amax1, amax2, amax3;
String command;
bool isCalibrated = false, isCapturing = true;

void setup() {
  pinMode(pot0, INPUT);
  pinMode(pot1, INPUT);
  pinMode(pot2, INPUT);
  pinMode(pot3, INPUT);

  analog1.setCoef(kp);
  analog1.setStep(step);
  analog2.setCoef(kp);
  analog2.setStep(step);
  analog3.setCoef(kp);
  analog3.setStep(step);
  analog4.setCoef(kp);
  analog4.setStep(step);

  gyro1.setCoef(kg);
  gyro1.setStep(10);
  gyro2.setCoef(kg);
  gyro2.setStep(10);
  gyro3.setCoef(kg);
  gyro3.setStep(10);
  gyro4.setCoef(kg);
  gyro4.setStep(10);

  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(54); //++
  mpu.setYGyroOffset(-21); //--
  mpu.setZGyroOffset(5);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
    Serial.println("Error!");
}

void calibration() {
  isCapturing = false;
  Serial.println("1 finger 3 sec 180 grad");
  delay(waitTime);
  Serial.println("calibration 1 finger 2 sec");
  for (int i = 0; i < 20; i++) {
    amin0 += analogRead(pot0);
    delay(100);
  }
  amin0 = (amin0 / 20) - 933;
  Serial.println("calibration 1 finger min end");

  Serial.println("1 finger 3 sec max");
  delay(waitTime);
  Serial.println("calibration 1 finger 2 sec");
  for (int i = 0; i < 20; i++) {
    amax0 += analogRead(pot0);
    delay(100);
  }
  amax0 = (amax0 / 20) - 933;
  Serial.println("calibration 1 finger max end");

  //////////////////////////////////////////////////////

  Serial.println("2 finger 3 sec 180 grad");
  delay(waitTime);
  Serial.println("calibration 2 finger 2 sec");
  for (int i = 0; i < 20; i++) {
    amin1 += analogRead(pot1);
    delay(100);
  }
  amin1 = (amin1 / 20) - 933;
  Serial.println("calibration 2 finger min end");

  Serial.println("2 finger 3 sec max");
  delay(waitTime);
  Serial.println("calibration 2 finger 2 sec");
  for (int i = 0; i < 20; i++) {
    amax1 += analogRead(pot1);
    delay(100);
  }
  amax1 = (amax1 / 20) - 933;
  Serial.println("calibration 2 finger max end");

  //////////////////////////////////////////////////////

  Serial.println("3 finger 3 sec 180 grad");
  delay(waitTime);
  Serial.println("calibration 3 finger 2 sec");
  for (int i = 0; i < 20; i++) {
    amin2 += analogRead(pot2);
    delay(100);
  }
  amin2 = (amin2 / 20) - 933;
  Serial.println("calibration 3 finger min end");

  Serial.println("3 finger 3 sec max");
  delay(waitTime);
  Serial.println("calibration 3 finger 2 sec");
  for (int i = 0; i < 20; i++) {
    amax2 += analogRead(pot2);
    delay(100);
  }
  amax2 = (amax2 / 20) - 933;
  Serial.println("calibration 3 finger max end");

  //////////////////////////////////////////////////////

  Serial.println("4 finger 3 sec 180 grad");
  delay(waitTime);
  Serial.println("calibration 4 finger 2 sec");
  for (int i = 0; i < 20; i++) {
    amin3 += analogRead(pot3);
    delay(100);
  }
  amin3 = (amin3 / 20) - 933;
  Serial.println("calibration 4 finger min end");

  Serial.println("4 finger 3 sec max");
  delay(waitTime);
  Serial.println("calibration 4 finger 2 sec");
  for (int i = 0; i < 20; i++) {
    amax3 += analogRead(pot3);
    delay(100);
  }
  amax3 = (amax3 / 20) - 933;
  Serial.println("calibration 4 finger max end");

  //////////////////////////////////////////////////////

  Serial.println(amin0);
  Serial.println(amin1);
  Serial.println(amin2);
  Serial.println(amin3);

  Serial.println(amax0);
  Serial.println(amax1);
  Serial.println(amax2);
  Serial.println(amax3);

  isCalibrated = true;
  isCapturing = true;
}

float findMedianN_optim(float newVal) {
  static float buffer[NUM_READ];  // статический буфер
  static byte count = 0;
  buffer[count] = newVal;
  if ((count < NUM_READ - 1) and (buffer[count] > buffer[count + 1])) {
    for (int i = count; i < NUM_READ - 1; i++) {
      if (buffer[i] > buffer[i + 1]) {
        float buff = buffer[i];
        buffer[i] = buffer[i + 1];
        buffer[i + 1] = buff;
      }
    }
  } else {
    if ((count > 0) and (buffer[count - 1] > buffer[count])) {
      for (int i = count; i > 0; i--) {
        if (buffer[i] < buffer[i - 1]) {
          float buff = buffer[i];
          buffer[i] = buffer[i - 1];
          buffer[i - 1] = buff;
        }
      }
    }
  }
  if (++count >= NUM_READ) count = 0;
  return buffer[(int)NUM_READ / 2];
}

int getAngle(int fnum) {
  int angle = 0;

  if (isCalibrated)
    switch (fnum) {
      case 1:
        for (int i = 0; i < 20; i++)
        {
          angle += analogRead(pot0);
          delay(10);
        }
        angle = constrain((angle / 20) - 933, amin0, amax0);
        break;
      case 2:
        for (int i = 0; i < 20; i++)
        {
          angle += analogRead(pot1);
          delay(10);
        }
        angle = constrain((angle / 20) - 933, amin1, amax1);
        break;
      case 3:
        for (int i = 0; i < 20; i++)
        {
          angle += analogRead(pot2);
          delay(10);
        }
        angle = constrain((angle / 20) - 933, amin2, amax2);
        break;
      case 4:
        for (int i = 0; i < 20; i++)
        {
          angle += analogRead(pot3);
          delay(10);
        }
        angle = constrain((angle / 20) - 933, amin3, amax3);
        break;
    }
  else
    switch (fnum) {
      case 1:
        angle = (int)analog1.filteredTime(analogRead(pot0)) - 1023;
        angle = constrain(angle, -90, 0);
        break;
      case 2:
        angle = (int)analog2.filteredTime(analogRead(pot1)) - 1023;
        angle = constrain(angle, -90, 0);
        break;
      case 3:
        angle = (int)analog3.filteredTime(analogRead(pot2)) - 1023;
        angle = constrain(angle, -90, 0);
        break;
      case 4:
        angle = (int)analog4.filteredTime(analogRead(pot3)) - 1023;
        angle = constrain(angle, -90, 0);
        break;
    }
  return angle;
}

void loop() {
  while (isCapturing == true) {
    Qdata();
  }
}

void SendQuaternion() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  Serial.print(gyro1.filteredTime(q.w), 4); Serial.print("_");
  Serial.print(gyro2.filteredTime(q.x), 4); Serial.print("_");
  Serial.print(gyro3.filteredTime(q.y), 4); Serial.print("_");
  Serial.println(gyro4.filteredTime(q.z), 4);
}

void SendEuler() {
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  Serial.print(euler[0] * 180 / M_PI); Serial.print("_");
  Serial.print(euler[1] * 180 / M_PI); Serial.print("_");
  Serial.println(euler[2] * 180 / M_PI);
}

void Qdata() {
  int mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // check if overflow
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    Serial.print(getAngle(1)); Serial.print("_");
    Serial.print(getAngle(2)); Serial.print("_");
    Serial.print(getAngle(3)); Serial.print("_");
    Serial.print(getAngle(4)); Serial.print("_");

    SendQuaternion();
  }
}

void Edata() {
  int mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // check if overflow
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    Serial.print(getAngle(1)); Serial.print("_");
    Serial.print(getAngle(2)); Serial.print("_");
    Serial.print(getAngle(3)); Serial.print("_");
    Serial.print(getAngle(4)); Serial.print("_");

    SendEuler();
  }
}