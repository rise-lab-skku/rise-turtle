/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */
/* Modifier: Hong-ryul Jung */

#ifndef TURTLEBOT3_MOTOR_DRIVER_H_
#define TURTLEBOT3_MOTOR_DRIVER_H_

#include "variant.h"
#include <DynamixelSDK.h>
#include "rise_define.h"

// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_POSITION         132

// Limit values (XM430-W210-T and XM430-W350-T)
#define LIMIT_X_MAX_VELOCITY            337     // MAX RPM is 77 when DXL is powered 12.0V
                                                // 77 / 0.229 (RPM) = 336.24454...

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4

#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0

#define DXL_LEFT_ID                     1       // ID of left motor
#define DXL_RIGHT_ID                    2       // ID of right motor

#define BAUDRATE                        1000000 // baurd rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define LEFT                            0
#define RIGHT                           1

#define VELOCITY_CONSTANT_VALUE         1263.632956882  // V = r * w = r     *        (RPM             * 0.10472)
                                                        //           = 0.033 * (0.229 * Goal_Velocity) * 0.10472
                                                        //
                                                        // Goal_Velocity = V * 1263.632956882

#define DEBUG_SERIAL  SerialBT2

class RiseMotorDriver
{
 public:
  RiseMotorDriver();
  ~RiseMotorDriver();
  bool init(void);
  void close(void);

  bool change_mode(uint8_t mode);
  bool setTorque(bool onoff);
  bool getTorque();
  int getMode();

  // origin
  bool readEncoder(int32_t &left_value, int32_t &right_value);

  // custom
  // bool readHwError(uint8_t &left_value, uint8_t &right_value);
  // bool readGoalPWM(float &left_value, float &right_value);
  // bool readGoalVel(float &left_value, float &right_value);
  bool readPresentPWM(int16_t &left_value, int16_t &right_value);
  bool readPresentVel(int32_t &left_value, int32_t &right_value);


  /////////////////////////////////////////////////////////

  bool writeVelocity(int64_t left_value, int64_t right_value);
  
  //bool writeWheelVelocity(float left_value, float right_value);
  bool writePWM(float* topic_value);
  
  bool controlMotor(const float wheel_separation, float* value);

 private:
  uint32_t baudrate_;
  float  protocol_version_;
  uint8_t left_wheel_id_;
  uint8_t right_wheel_id_;
  bool torque_=false;
  int mode_=3;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
  dynamixel::GroupSyncWrite *groupSyncWritePWM_;

  dynamixel::GroupSyncRead *groupSyncReadEncoder_;      // Present Position
  // dynamixel::GroupSyncRead *groupSyncReadHwError_;
  // dynamixel::GroupSyncRead *groupSyncReadGoalPWM_;
  // dynamixel::GroupSyncRead *groupSyncReadGoalVel_;
  dynamixel::GroupSyncRead *groupSyncReadPresentPWM_;
  dynamixel::GroupSyncRead *groupSyncReadPresentVel_;   // Present Velocity

  bool change_controlTable_1Byte(uint16_t address, uint8_t left_data, uint8_t right_data);
  // bool change_controlTable_1Byte(uint8_t id, uint16_t address, uint8_t data);

  bool read_table_(
    int32_t &left_value, int32_t &right_value,
    dynamixel::GroupSyncRead *groupSync_, const uint16_t address, const uint16_t data_length);
};

#endif // TURTLEBOT3_MOTOR_DRIVER_H_
