// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//       - and altered FIFO retrieval sequence to avoid using blocking code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

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

//#include "mpr121.h"
//#include <Wire.h>

#include "Adafruit_MPR121.h"
Adafruit_MPR121 cap = Adafruit_MPR121();
#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif

#include "Simple_MPU6050.h"
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW
#define Three_Axis_Quaternions 3
#define Six_Axis_Quaternions 6  // Default
Simple_MPU6050 mpu(Six_Axis_Quaternions);

int irqpin = 4;           // Digital 2
boolean touchStates[12];  //to keep track of the previous touch states


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
#define OUTPUT_READABLE_YAWPITCHROLL

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
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 2   // use pin 2 on Arduino Uno & most boards
#define INTERRUPT_PIN1 3  // use pin 2 on Arduino Uno & most boards

#define LED_PIN 13  // (Arduino is 13, Teensy is 11, Teensy++ is 6)

bool blinkState = false;

// MPU control/status vars
bool flip = false;
// orientation/motion vars
Quaternion q, q1;  // [w, x, y, z]         quaternion container
uint16_t currtouched = 0;
uint16_t lasttouched = 0;

#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis()) // (BLACK BOX) Ya, don't complain that I used "for(;;){}" instead of "if(){}" for my Blink Without Delay Timer macro. It works nicely!!!

void update_values (int16_t *gyro, int16_t *accel, int32_t *quat) {
  uint8_t WhoAmI;
  mpu.WHO_AM_I_READ_WHOAMI(&WhoAmI);
  if(digitalRead(8) == 0){
    mpu.GetQuaternion(&q, quat);
  }
  else{
    mpu.GetQuaternion(&q1, quat);
  }
  //mpu.GetQuaternion(&q, quat);
  //Serial.printfloatx(F("quat w")  , q.w, 9, 4, F(",   ")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
  //Serial.printfloatx(F("x")       , q.x, 9, 4, F(",   "));
  //Serial.printfloatx(F("y")       , q.y, 9, 4, F(",   "));
  //Serial.printfloatx(F("z")       , q.z, 9, 4, F("\n"));
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  // initialize serial communication
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println(F("Start:"));
  mpu.begin();
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS);
  mpu.Set_DMP_Output_Rate_Hz(20);           // Set the DMP output rate from 200Hz to 5 Minutes.
  //mpu.Set_DMP_Output_Rate_Seconds(1);   // Set the DMP output rate in Seconds
  //mpu.Set_DMP_Output_Rate_Minutes(5);    // Set the DMP output rate in Minute
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8, HIGH);
  digitalWrite(9, LOW);
  mpu.CalibrateMPU().load_DMP_Image();// Does it all for you with Calibration
  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);
  mpu.CalibrateMPU().Enable_Reload_of_DMP(Three_Axis_Quaternions).load_DMP_Image();// Does it all for you with Calibration
  mpu.on_FIFO(update_values);
  Serial.println("");

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  //Serial.println(mpu1.testConnection() ? F("MPU6050_1 connection successful") : F("MPU6050_1 connection failed"));

  // wait for ready
  Serial.println("a");
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read())
    ;  // empty buffer
  while (!Serial.available())
    ;  // wait for data
  while (Serial.available() && Serial.read())
    ;  // empty buffer again


  //Touch Sensor Setup
  pinMode(irqpin, INPUT);
  digitalWrite(irqpin, HIGH);  //enable pullup resistor
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  //Wire.begin();

  //mpr121_setup();
  

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  //readTouchInputs();

  // if programming failed, don't try to do anything
  if(flip){
    digitalWrite(8, HIGH);
    //digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
    //digitalWrite(9, LOW);
  }
  else{
    digitalWrite(8, LOW);
    //digitalWrite(8, HIGH);
    digitalWrite(9, HIGH);
    //digitalWrite(9, LOW);
  }
  flip = !flip;
  mpu.dmp_read_fifo(0);// Must be in loop
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);  

  //Touch sensor code
  
  currtouched = cap.touched();
  
    
  for (uint8_t i=0; i<12; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      //Serial.print(i); Serial.println(" touched");
      touchStates[i] = 1;
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      //Serial.print(i); Serial.println(" released");
      touchStates[i] = 0;
    }
  }
  lasttouched = currtouched;

  spamtimer(1) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    for(int i =0; i < 8; i++) {
      Serial.print(touchStates[i]);
      Serial.print(",");
    }
  
    Serial.print(q.w);
    Serial.print(",");
    Serial.print(q.x);
    Serial.print(",");
    Serial.print(q.y);
    Serial.print(",");
    Serial.print(q.z);
    Serial.print(",");
    Serial.print(q1.w);
    Serial.print(",");
    Serial.print(q1.x);
    Serial.print(",");
    Serial.print(q1.y);
    Serial.print(",");
    Serial.print(q1.z);
    
    Serial.println("\n");
  }
  delay(20);

  //String sendvalue_quaternion = String(q.w) + "," + String(q.x) + "," + String(q.y) + ","+ String(q.z) + "," + String(q1.w) + "," + String(q1.x) + "," + String(q1.y) + ","+ String(q1.z);  // +"," + touchStates[0] ;
  //String sendvalue = String(yaw1) + "," + String(pitch1) + "," + String(roll1) + "," + String(yaw2) + "," + String(pitch2) + "," + String(roll2); // +"," + touchStates[0] ;
  //Serial.println(sendvalue_quaternion);
}

/*void readTouchInputs(){
  if(!checkInterrupt()){

    //read the touch state from the MPR121
    Wire.requestFrom(0x5A,2); 

    byte LSB = Wire.read();
    byte MSB = Wire.read();

    uint16_t touched = ((MSB << 8) | LSB); //16bits that make up the touch states


    for (int i=0; i < 12; i++){  // Check what electrodes were pressed
      if(touched & (1<<i)){

        if(touchStates[i] == 0){
          //pin i was just touched
          Serial.print("pin ");
          Serial.print(i);
          Serial.println(" was just touched");

        }else if(touchStates[i] == 1){
          //pin i is still being touched
        }  

        touchStates[i] = 1;      
      }else{
        if(touchStates[i] == 1){
          Serial.print("pin ");
          Serial.print(i);
          Serial.println(" is no longer being touched");

          //pin i is no longer being touched
       }

        touchStates[i] = 0;
      }

    }

  }
}



void mpr121_setup(void){

  set_register(0x5A, ELE_CFG, 0x00); 

  // Section A - Controls filtering when data is > baseline.
  set_register(0x5A, MHD_R, 0x01);
  set_register(0x5A, NHD_R, 0x01);
  set_register(0x5A, NCL_R, 0x00);
  set_register(0x5A, FDL_R, 0x00);

  // Section B - Controls filtering when data is < baseline.
  set_register(0x5A, MHD_F, 0x01);
  set_register(0x5A, NHD_F, 0x01);
  set_register(0x5A, NCL_F, 0xFF);
  set_register(0x5A, FDL_F, 0x02);

  // Section C - Sets touch and release thresholds for each electrode
  set_register(0x5A, ELE0_T, TOU_THRESH);
  set_register(0x5A, ELE0_R, REL_THRESH);

  set_register(0x5A, ELE1_T, TOU_THRESH);
  set_register(0x5A, ELE1_R, REL_THRESH);

  set_register(0x5A, ELE2_T, TOU_THRESH);
  set_register(0x5A, ELE2_R, REL_THRESH);

  set_register(0x5A, ELE3_T, TOU_THRESH);
  set_register(0x5A, ELE3_R, REL_THRESH);

  set_register(0x5A, ELE4_T, TOU_THRESH);
  set_register(0x5A, ELE4_R, REL_THRESH);

  set_register(0x5A, ELE5_T, TOU_THRESH);
  set_register(0x5A, ELE5_R, REL_THRESH);

  set_register(0x5A, ELE6_T, TOU_THRESH);
  set_register(0x5A, ELE6_R, REL_THRESH);

  set_register(0x5A, ELE7_T, TOU_THRESH);
  set_register(0x5A, ELE7_R, REL_THRESH);

  set_register(0x5A, ELE8_T, TOU_THRESH);
  set_register(0x5A, ELE8_R, REL_THRESH);

  set_register(0x5A, ELE9_T, TOU_THRESH);
  set_register(0x5A, ELE9_R, REL_THRESH);

  set_register(0x5A, ELE10_T, TOU_THRESH);
  set_register(0x5A, ELE10_R, REL_THRESH);

  set_register(0x5A, ELE11_T, TOU_THRESH);
  set_register(0x5A, ELE11_R, REL_THRESH);

  // Section D
  // Set the Filter Configuration
  // Set ESI2
  set_register(0x5A, FIL_CFG, 0x04);

  // Section E
  // Electrode Configuration
  // Set ELE_CFG to 0x00 to return to standby mode
  set_register(0x5A, ELE_CFG, 0x0C);  // Enables all 12 Electrodes


  // Section F
  // Enable Auto Config and auto Reconfig
  /*set_register(0x5A, ATO_CFG0, 0x0B);
  set_register(0x5A, ATO_CFGU, 0xC9);  // USL = (Vdd-0.7)/vdd*256 = 0xC9 @3.3V   set_register(0x5A, ATO_CFGL, 0x82);  // LSL = 0.65*USL = 0x82 @3.3V
  set_register(0x5A, ATO_CFGT, 0xB5);  // Target = 0.9*USL = 0xB5 @3.3V

  set_register(0x5A, ELE_CFG, 0x0C);

}*/



boolean checkInterrupt(void) {
  return digitalRead(irqpin);
}

void set_register(int address, unsigned char r, unsigned char v){
    Wire.beginTransmission(address);
    Wire.write(r);
    Wire.write(v);
    Wire.endTransmission();
}
