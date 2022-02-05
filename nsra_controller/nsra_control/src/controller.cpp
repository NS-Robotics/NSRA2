/*****************************************************************************
*  BSD 3-Clause License
* 
*  Copyright (c) 2020, Noa Sendlhofer
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
* 
*  1. Redistributions of source code must retain the above copyright notice, this
*     list of conditions and the following disclaimer.
* 
*  2. Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
* 
*  3. Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

/* Author: Noa Sendlhofer
   Desc:   Part of the NSRA Control stack. Teensy ODrive UART controller.
*/

#include <Arduino.h>
#include <ODriveArduino.h>
#include <HardwareSerial.h>
#include <ArduinoQueue.h>
#include <CRC32.h>
#include <Base64.h>

#define QUEUE_SIZE 3
#define FRQ 100

#define VEL_LIMIT_A1 3.0
#define VEL_LIMIT_A2 4.0
#define VEL_LIMIT_A3 3.0
#define VEL_LIMIT_A4 3.0
#define VEL_LIMIT_A5 3.0
#define VEL_LIMIT_A6 3.0

#define VEL_DIFF_A1 3.0
#define VEL_DIFF_A2 2.0
#define VEL_DIFF_A3 1.0
#define VEL_DIFF_A4 1.0
#define VEL_DIFF_A5 1.0
#define VEL_DIFF_A6 1.0

#define GRIPPER_PIN 2

template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

HardwareSerial& odrive_serial0 = Serial3;
HardwareSerial& odrive_serial1 = Serial1;
HardwareSerial& odrive_serial2 = Serial2;

ODriveArduino odrv0(odrive_serial0);
ODriveArduino odrv1(odrive_serial1);
ODriveArduino odrv2(odrive_serial2);

bool serialFlag = false;

IntervalTimer ctrl_loop_timer;

bool queueFlag = false;
bool controlFlag = false;

volatile bool gripper_enabled = false;

struct pos {
    volatile float axis1;
    volatile float axis2;
    volatile float axis3;
    volatile float axis4;
    volatile float axis5;
    volatile float axis6;
    pos(): axis1(0), axis2(0), axis3(0), axis4(0), axis5(0), axis6(0) {}
};
ArduinoQueue<pos> queue(50);

pos last;

float calc_vel(float last, float now, float next, float diff)
{
  return (abs(last - now) + abs(now - next)) / 2.0 * (float)FRQ + diff;
}

void update() {
  if(queueFlag && controlFlag)
  {
    
    pos now = queue.dequeue();
    pos next = queue.getHead();

    float axis1_vel = calc_vel(last.axis1, now.axis1, next.axis1, VEL_DIFF_A1);
    if(axis1_vel < VEL_LIMIT_A1)
      odrive_serial1 << "w axis" << 0 << ".controller.config.vel_limit " << (float)VEL_LIMIT_A1 << '\n';
    else
      odrive_serial1 << "w axis" << 0 << ".controller.config.vel_limit " << axis1_vel << '\n';

    float axis2_vel = calc_vel(last.axis2, now.axis2, next.axis2, VEL_DIFF_A2);
    if (axis2_vel < VEL_LIMIT_A2)
      odrive_serial0 << "w axis" << 0 << ".controller.config.vel_limit " << (float)VEL_LIMIT_A2 << '\n';
    else
      odrive_serial0 << "w axis" << 0 << ".controller.config.vel_limit " << axis2_vel << '\n';

    float axis3_vel = calc_vel(last.axis3, now.axis3, next.axis3, VEL_DIFF_A3);
    if (axis3_vel < VEL_LIMIT_A3)
      odrive_serial0 << "w axis" << 1 << ".controller.config.vel_limit " << (float)VEL_LIMIT_A3 << '\n';
    else
      odrive_serial0 << "w axis" << 1 << ".controller.config.vel_limit " << axis3_vel << '\n';

    float axis4_vel = calc_vel(last.axis4, now.axis4, next.axis4, VEL_DIFF_A4);
    if (axis4_vel < VEL_LIMIT_A4)
      odrive_serial2 << "w axis" << 0 << ".controller.config.vel_limit " << (float)VEL_LIMIT_A4 << '\n';
    else
      odrive_serial2 << "w axis" << 0 << ".controller.config.vel_limit " << axis4_vel << '\n';

    float axis5_vel = calc_vel(last.axis5, now.axis5, next.axis5, VEL_DIFF_A5);
    if (axis5_vel < VEL_LIMIT_A5)
      odrive_serial2 << "w axis" << 1 << ".controller.config.vel_limit " << (float)VEL_LIMIT_A5 << '\n';
    else
      odrive_serial2 << "w axis" << 1 << ".controller.config.vel_limit " << axis5_vel << '\n';

    float axis6_vel = calc_vel(last.axis6, now.axis6, next.axis6, VEL_DIFF_A6);
    if (axis6_vel < VEL_LIMIT_A6)
      odrive_serial1 << "w axis" << 1 << ".controller.config.vel_limit " << (float)VEL_LIMIT_A6 << '\n';
    else
      odrive_serial1 << "w axis" << 1 << ".controller.config.vel_limit " << axis6_vel << '\n';

    odrv1.SetPosition(0, (float)now.axis1);
    odrv0.SetPosition(0, (float)now.axis2);
    odrv0.SetPosition(1, (float)now.axis3);
    odrv2.SetPosition(0, (float)now.axis4);
    odrv2.SetPosition(1, (float)now.axis5);
    odrv1.SetPosition(1, (float)now.axis6);

    last = now;
    
  } else if(queue.itemCount() >= QUEUE_SIZE && !queueFlag){
    queueFlag = true;
  }
  
  if(queue.itemCount() == 0 && queueFlag){
    queueFlag = false;
  }
  
}

void serial_interrupt() {

  char in_bytes[24];
  CRC32 crc;
  
  while (Serial.available())
  {
    in_bytes[0] = Serial.read();
    if (in_bytes[0] == 10) { break; }
  }

  for (int n = 0; n < 24; n++)
  {
    in_bytes[n] = Serial.read();
  }

  char dec_string[17];

  Base64.decode(dec_string, in_bytes, 24);

  uint32_t checksum = crc.calculate(dec_string, 13);

  uint32_t rec_checksum;

  rec_checksum = dec_string[16];
  rec_checksum = rec_checksum << 8;
  rec_checksum = rec_checksum | dec_string[15];
  rec_checksum = rec_checksum << 8;
  rec_checksum = rec_checksum | dec_string[14];
  rec_checksum = rec_checksum << 8;
  rec_checksum = rec_checksum | dec_string[13];

  if(rec_checksum == checksum) {

    if(!controlFlag) {
      serialFlag = true;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    
    pos n;
  
    n.axis1 = ((uint16_t)((dec_string[1] << 8) | dec_string[0]) - 32000);
    n.axis2 = ((uint16_t)((dec_string[3] << 8) | dec_string[2]) - 32000);
    n.axis3 = ((uint16_t)((dec_string[5] << 8) | dec_string[4]) - 32000);
    n.axis4 = ((uint16_t)((dec_string[7] << 8) | dec_string[6]) - 32000);
    n.axis5 = ((uint16_t)((dec_string[9] << 8) | dec_string[8]) - 32000);
    n.axis6 = ((uint16_t)((dec_string[11] << 8) | dec_string[10]) - 32000);
    gripper_enabled = (uint8_t)dec_string[12];
  
    queue.enqueue(n);

  } else if(controlFlag) {

    while(queue.itemCount() != 0) {
      queue.dequeue();
    }

    controlFlag = false;
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(GRIPPER_PIN, LOW);
  }

}

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GRIPPER_PIN, OUTPUT);
  digitalWrite(GRIPPER_PIN, LOW);

  odrive_serial0.begin(115200);
  odrive_serial1.begin(115200);
  odrive_serial2.begin(115200);

  delay(2000);

  while (!Serial) {
    delay(1);
  }

  ctrl_loop_timer.begin(update, 1000000/FRQ);

}

void loop() {
  
  if(Serial.available() > 24) {
    serial_interrupt();
  }

  if(serialFlag) {
    serialFlag = false;
    controlFlag = true;
  }

  if(controlFlag) {
    digitalWrite(GRIPPER_PIN, gripper_enabled);
  }

  if(!Serial && (controlFlag || serialFlag)) {

    while(queue.itemCount() != 0) {
      queue.dequeue();
    }

    controlFlag = false;
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(GRIPPER_PIN, LOW);
  }
  
}