
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//     2012-06-20 - improved FIFO overflow handling and simplified read process
//     2012-06-19 - completely rearranged DMP initialization code and simplification
//     2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//     2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//     2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                - add 3D math helper file to DMP6 example sketch
//                - add Euler output and Yaw/Pitch/Roll output formats
//     2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//     2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//     2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
// #define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
// #define OUTPUT_TEAPOT

#define OUTPUT_TOUCH_VISON



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// touchVision Definitions
float dt = float(1.0 / 100.0); // This variable assumes the sampling frequency is 200 Hz.
float fifo_sample_period = float(1.0 / 100.0);
float LSB_g = float(16384.0 / 2.0); // This variable assumes the LSB/g  rjabaker -- is this incorrect?, should it be divided by two
float accel_g = 9.832; // m/s^2
int sample_count = 0;

float z_average = 0.0; // We know that the z axis stays relatively constant, so during cal, find average and use as offset.

int16_t accel_x[21];
int16_t accel_y[21];
int16_t accel_z[21];

float vel_x[21];
float vel_y[21];
float vel_z[21];

float pos_x[21];
float pos_y[21];
float pos_z[21];

float pressure_normalizer = 200;

int touchSensorPin = 0;
float pressureValue = 0;


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
    Wire.begin();

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    //Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize touchVision
    // Serial.println(F("Initializing touchVision Alpha..."));
    setup_touchVision();
    
    // initialize device
    // Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    // Serial.println(F("Testing device connections..."));
    // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    // Serial.println("00000"); // begin, rjabaker
    // while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    // Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        // Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
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
        // Serial.print(F("DMP Initialization failed (code "));
        // Serial.print(devStatus);
        // Serial.println(F(")"));
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

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
        
        #ifdef OUTPUT_TOUCH_VISON
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            
            sample_count += 1;
            push_fifo(accel_x, aaWorld.x, 21);
            push_fifo(accel_y, aaWorld.y, 21);
            push_fifo(accel_z, sub_zero(aaWorld.z, z_average), 21);
            
            if ( float(sample_count) * fifo_sample_period > 25 ) {
              calc_displacement(accel_x, vel_x, pos_x, 21);
              calc_displacement(accel_y, vel_y, pos_y, 21);
              
              calc_displacement(accel_z, vel_z, pos_z, 21);
            }
            else
            {
               // z_average = float((z_average * float(sample_count - 1) + float(accel_z[20])) / float(sample_count));
               z_average = 2010;
            }
            
            if ( sample_count % 25 == 0 ) {
              pressureValue = analogRead(touchSensorPin);
        /* 
              Serial.print("Runtime: \t");
              Serial.println(float(sample_count) * fifo_sample_period);
              Serial.print("Timestamp: \t");
              Serial.println(float(sample_count) * dt);
              Serial.print("Pressure:\t");
              Serial.println(pressureValue);
              Serial.print("p(x, y z)\t");
              Serial.print(pos_x[12], 5);
              Serial.print("\t");
              Serial.print(pos_y[12], 5);
              Serial.print("\t");
              Serial.println(pos_z[12], 5);
              Serial.print("a(x, y z)\t");
              Serial.print(accel_x[12], 5);
              Serial.print("\t");
              Serial.print(accel_y[12], 5);
              Serial.print("\t");
              Serial.println(accel_z[12], 5);
              Serial.print("v(x, y z)\t");
              Serial.print(vel_x[12], 5);
              Serial.print("\t");
              Serial.print(vel_y[12], 5);
              Serial.print("\t");
              Serial.println(vel_z[12], 5);
              */
           
              char x[8];
              char y[8];
              char z[8];
              char pressure[8];
              char time[16];
              
              
              String px=dtostrf(pos_x[12],2, 5, x);
              String py=dtostrf(pos_y[12],2, 5, y);
              String pz=dtostrf(pos_z[12],2, 5, z);
              String pre=dtostrf(pressureValue / pressure_normalizer, 2, 5, pressure);
              String ts=dtostrf(float(sample_count)*dt, 2, 5, time);
              
              String posx=(px);
              String posy=(py);
              String posz=(pz);
              String pressureVal=(pre);
              String timeStamp= (ts);
              
              Serial.println(timeStamp + " " + posx + " "+ posy + " " + posz + " " + pressureVal);
            }
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void setup_touchVision(){
 pinMode(touchSensorPin, INPUT);
  
 initialize_fifo(accel_x, 21);
 initialize_fifo(accel_y, 21);
 initialize_fifo(accel_z, 21);
 
 initialize_fifo(vel_x, 21);
 initialize_fifo(vel_y, 21);
 initialize_fifo(vel_z, 21);
 
 initialize_fifo(pos_x, 21);
 initialize_fifo(pos_y, 21);
 initialize_fifo(pos_z, 21);
}

void initialize_fifo(int16_t *fifo, int fsize) {
  int i = 0;
  for (i = 0; i < fsize; i++) {
    fifo[i] = 0;
  } 
}

void initialize_fifo(float *fifo, int fsize) {
  int i = 0;
  for (i = 0; i < fsize; i++) {
    fifo[i] = 0;
  } 
}

void push_fifo(int16_t *fifo, int16_t newValue, int fsize) {
 // This is not actually a fifo. We are putting it at the end of the queue
 // and pushing.
 int i = 0;
 for (i = 0; i < fsize; i++) {
  if (i == fsize - 1) {
   fifo[i] = newValue; 
  }
  else {
   fifo[i] = fifo[i + 1]; 
  }
 }
}

void push_fifo(float *fifo, float newValue, int fsize) {
 // This is not actually a fifo. We are putting it at the end of the queue
 // and pushing.
 int i = 0;
 for (i = 0; i < fsize; i++) {
  if (i == fsize - 1) {
   fifo[i] = newValue; 
  }
  else {
   fifo[i] = fifo[i + 1]; 
  }
 }
}

void calc_displacement(int16_t *a, float *v, float *p, int fsize) {
  int trigger_length = 4;
  float trigger_sum = 0.0;
  float kill_th_low = 0.01;
  float kill_th_high = 10.0; // greater than the acceleration of gravity
  
  float vx = 0;
  float px = 0;
  float ax = float(a[fsize - 1]) * accel_g / LSB_g; // m/s^2
  if (sample_count == 1) {
    // First sample
    if (abs(ax) < kill_th_low || abs(ax) > kill_th_high) { 
      // Kill ridiculous values
      ax = 0;
      a[fsize - 1] = 0;
    }
    vx = 0;
    px = p[fsize - 1] + vx * dt + 0.5 * ax * dt * dt;
  }
  else
  {
    if (abs(ax) < kill_th_low || abs(ax) > kill_th_high) { 
      // Kill ridiculous values
      ax = 0;
      a[fsize - 1] = 0;
    }
    
    trigger_sum = 0;
    for(int ii = fsize - 1; ii >= fsize - 1 - trigger_length; ii--) {
      trigger_sum += abs( float(a[ii])  * accel_g / LSB_g );
     }
    
    vx = v[fsize - 1] + ax * dt;
    if (trigger_sum == 0) { vx = 0; }
    px = p[fsize - 1] + vx * dt + 0.5 * ax * dt * dt;
  }
  
  push_fifo(v, vx, fsize);
  push_fifo(p, px, fsize);
}

float sub_zero(int16_t numberA, float numberB) {
  float number = 0;
 if(numberB > numberA) {
  return number;
 }
 else
 {
  number = float(numberA) - numberB;
 }
 return number;
} 
