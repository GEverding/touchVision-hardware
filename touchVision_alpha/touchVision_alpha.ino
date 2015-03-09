
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

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

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
float dt = float(0.02272727273); // float(1.0 / 100.0);
float fifo_sample_period = dt; // float(1.0 / 100.0);
// float LSB_g = float(16384.0 / 2.0); // This variable assumes the LSB/g  rjabaker -- is this incorrect?, should it be divided by two
float accel_g = 9.832; // m/s^2
int sample_count = 0;
int send_rate = 25; // Send once every 25 samples
int send_position_threshold = 25; // Only send data after 25 samples

int touchSensorPin = 0;

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
    Serial.begin(115200); // rjabaker - Is this baud rate too high for the slow processor?
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    setup_touchVision();    
    mpu.initialize();
    while (Serial.available() && Serial.read()); // empty buffer
    while (Serial.available() && Serial.read()); // empty buffer again
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt...")); // rjabaker - Do we need this line?
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
    }

    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    int index = 0;
    int16_t raw_data[3];
    String output[1];
    float pressure_value = sample_pressure();
    for (index = 0; index < 1; index++) {
      sample_accelerometer(raw_data);
      sample_count = sample_count + 1;
      add_data_sample_to_output(output, raw_data, pressure_value, index);
    }
    send_data(output, 1);
}

void send_data(String *output, int length) {
  String to_go = "";
  for (int index = 0; index < length; index++) {
    to_go = to_go + output[index];
    if(index < length - 1) {
      to_go = to_go + "<>";
    }
  }
  Serial.println(to_go);
}

void add_data_sample_to_output(String *output, int16_t *raw_data, float pressure_value, int index) {
    char x[16];
    char y[16];
    char z[16];
    char pressure[8];
    char time[16];
    
    String px=dtostrf(raw_data[0],2, 5, x);
    String py=dtostrf(raw_data[1],2, 5, y);
    String pz=dtostrf(raw_data[2],2, 5, z);
    String pre=dtostrf(pressure_value, 2, 5, pressure);
    String ts=dtostrf(float(sample_count)*dt, 2, 5, time);
    
    String posx=(px);
    String posy=(py);
    String posz=(pz);
    String pressureVal=(pre);
    String timeStamp= (ts);

    output[index] =  posx + " " + posy + " " + posz + " " + pressureVal + " " + timeStamp;

}

void setup_touchVision(){
 pinMode(touchSensorPin, INPUT);
  
 // initialize_fifo(accel_x, 21);
 // initialize_fifo(accel_y, 21);
 // initialize_fifo(accel_z, 21);
 
 // initialize_fifo(vel_x, 21);
 // initialize_fifo(vel_y, 21);
 // initialize_fifo(vel_z, 21);
 
 // initialize_fifo(pos_x, 21);
 // initialize_fifo(pos_y, 21);
 // initialize_fifo(pos_z, 21);

 // initialize_fifo(particles_filtered_x, (int)window_size[0]);
 // initialize_fifo(particles_filtered_y, (int)window_size[1]);
 // initialize_fifo(particles_filtered_z, (int)window_size[2]);
}

void sample_accelerometer(int16_t *raw_data) {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // rjabaker - Nothing needs to happen here right now
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // rjabaker - Do we need error handling here? I am assuming this NEVER happens.
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!")); // rjabaker - Is this necessary?
        
        raw_data[0] = 0.0;
        raw_data[1] = 0.0;
        raw_data[2] = 0.0;

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        raw_data[0] = aaWorld.x;
        raw_data[1] = aaWorld.y;
        raw_data[2] = aaWorld.z;
    }
}

float sample_pressure() {
    return analogRead(touchSensorPin);
}

// void send_data(int send_rate, int sample_count) {
//   // rjabaker - I'm hardcoding send_data for now, because it is throwing an error...
//   float raw_pressure_value;

//   if (sample_count % 25 == 0) {
//     raw_pressure_value = sample_pressure(); // rjabaker - I know it is bad form to do this in here, but this operation is slow, so we only do it every 25 samples.
//     add_pressure_data(raw_pressure_value);
 
//     char x[8];
//     char y[8];
//     char z[8];
//     char pressure[8];
//     char time[16];
    
    
//     String px=dtostrf(pos_x[12],2, 5, x);
//     String py=dtostrf(pos_y[12],2, 5, y);
//     String pz=dtostrf(pos_z[12],2, 5, z);
//     String pre=dtostrf(normalized_pressure_value, 2, 5, pressure);
//     String ts=dtostrf(float(sample_count)*dt, 2, 5, time);
    
//     String posx=(px);
//     String posy=(py);
//     String posz=(pz);
//     String pressureVal=(pre);
//     String timeStamp= (ts);
    
//     Serial.println(timeStamp + " " + posx + " "+ posy + " " + posz + " " + pressureVal);

//     blinkState = !blinkState;
//     digitalWrite(LED_PIN, blinkState);
//   }
// }

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
