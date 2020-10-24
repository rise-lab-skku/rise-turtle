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

#include "../../include/rise_turtlebot3/rise_motor_driver.h"


RiseMotorDriver::RiseMotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  left_wheel_id_(DXL_LEFT_ID),
  right_wheel_id_(DXL_RIGHT_ID),
  torque_(false)
{
}

RiseMotorDriver::~RiseMotorDriver()
{
  close();
}

bool RiseMotorDriver::init(void)
{
  DEBUG_SERIAL.begin(57600);
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort() == false)
  {
    DEBUG_SERIAL.println("Failed to open port(Motor Driver)");
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_) == false)
  {
    DEBUG_SERIAL.println("Failed to set baud rate(Motor Driver)");
    return false;
  }

  //////////////////////////////////////////
  // Enable Dynamixel Torque
  // Initialize EEPROM area: Velocity mode
  setTorque(false);
  // 0 : CCW +
  // 1 : CW +
  bool drive_result = change_controlTable_1Byte(10, 0, 1);
  change_mode(MODE_VELOCITY);
  //////////////////////////////////////////

  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(
    portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncWritePWM_ = new dynamixel::GroupSyncWrite(
    portHandler_, packetHandler_, 100, 2);

  // origin
  groupSyncReadEncoder_     = new dynamixel::GroupSyncRead( // Present Position(132)
    portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  // custom
  // groupSyncReadHwError_     = new dynamixel::GroupSyncRead(
  //   portHandler_, packetHandler_, 70, 1);
  // groupSyncReadGoalPWM_     = new dynamixel::GroupSyncRead(
  //   portHandler_, packetHandler_, 100, 2);
  // groupSyncReadGoalVel_     = new dynamixel::GroupSyncRead(
  //   portHandler_, packetHandler_, 104, 4);
  groupSyncReadPresentPWM_  = new dynamixel::GroupSyncRead(
    portHandler_, packetHandler_, 124, 2);
  groupSyncReadPresentVel_  = new dynamixel::GroupSyncRead(
    portHandler_, packetHandler_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);



  DEBUG_SERIAL.println("Success to init Motor Driver");
  return true;
}

void RiseMotorDriver::close(void)
{
  // Disable Dynamixel Torque
  setTorque(false);

  // Close port
  portHandler_->closePort();
  DEBUG_SERIAL.end();
}




bool RiseMotorDriver::change_mode(uint8_t mode)
{
  if(mode == MODE_VELOCITY) {}
  else if(mode == MODE_PWM) {}
  else { return false; }

  if(!setTorque(false)) {
    if(!setTorque(false)) {
      setTorque(false); }};
  // bool drive_result = change_controlTable_1Byte(10, 0, 1);
  if(torque_ == false)
  {
    bool op_result = change_controlTable_1Byte(11, mode, mode);
    // bool drive_result = (
    //   change_controlTable_1Byte(DXL_LEFT_ID, 10, 0) || 
    //   change_controlTable_1Byte(DXL_RIGHT_ID, 10, 0));
    // bool op_result = (
    //   change_controlTable_1Byte(DXL_LEFT_ID, 11, mode) || 
    //   change_controlTable_1Byte(DXL_RIGHT_ID, 11, mode));
    // setTorque(true);
    if (op_result) {
      if (setTorque(true)) {
        mode_ = (int)mode;
        return true;
      }
    }
  }
  DEBUG_SERIAL.println("Motor mode change succeeded");
  // return (drive_result || op_result);
  return false;
}

bool RiseMotorDriver::setTorque(bool onoff)
{
  if (change_controlTable_1Byte(ADDR_X_TORQUE_ENABLE, onoff, onoff))
  {
    torque_ = onoff;
    return true;
  }
  else
  {
    return false;
  }
  // return (
  //   change_controlTable_1Byte(DXL_LEFT_ID, ADDR_X_TORQUE_ENABLE, onoff) || 
  //   change_controlTable_1Byte(DXL_RIGHT_ID, ADDR_X_TORQUE_ENABLE, onoff));
}

bool RiseMotorDriver::getTorque()
{
  return torque_;
}

int RiseMotorDriver::getMode()
{
  // int success = 0;
  // uint8_t right_data = 0;
  // uint8_t left_data = 0;
  // uint8_t error = 0;
  // int right_result = packetHandler_->read1ByteTxRx(portHandler_, DXL_RIGHT_ID, 11, &right_data, &error);
  // int left_result = packetHandler_->read1ByteTxRx(portHandler_, DXL_LEFT_ID, 11, &left_data, &error);
  
  // int data = ((int)left_data)*100 + ((int)right_data);
  // return data;
  return mode_;
}

// int Protocol2PacketHandler::read1ByteTxRx(PortHandler *port, uint8_t id, 
// uint16_t address, uint8_t *data, uint8_t *error)
// {
//   uint8_t data_read[1] = {0};
//   int result = readTxRx(port, id, address, 1, data_read, error);
//   if (result == COMM_SUCCESS)
//     *data = data_read[0];
//   return result;
// }


// bool RiseMotorDriver::change_controlTable_1Byte(uint8_t id, uint16_t address, uint8_t data)
// {
//   uint8_t dxl_error = 0;
//   int dxl_comm_result = COMM_TX_FAIL;

//   dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, address, data, &dxl_error);
//   if(dxl_comm_result != COMM_SUCCESS)
//   {
//     Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
//     return false;
//   }
//   else if(dxl_error != 0)
//   {
//     Serial.println(packetHandler_->getRxPacketError(dxl_error));
//     return false;
//   }
//   return true;
// }

bool RiseMotorDriver::change_controlTable_1Byte(uint16_t address, uint8_t left_data, uint8_t right_data)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_RIGHT_ID, address, right_data, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }
  
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_LEFT_ID, address, left_data, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }


  return true;
}






////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


// bool RiseMotorDriver::readEncoder(int32_t &left_value, int32_t &right_value)
// {
//   int dxl_comm_result = COMM_TX_FAIL;              // Communication result
//   bool dxl_addparam_result = false;                // addParam result
//   bool dxl_getdata_result = false;                 // GetParam result

//   // Set parameter
//   dxl_addparam_result = groupSyncReadEncoder_->addParam(left_wheel_id_);
//   if (dxl_addparam_result != true)
//     return false;

//   dxl_addparam_result = groupSyncReadEncoder_->addParam(right_wheel_id_);
//   if (dxl_addparam_result != true)
//     return false;

//   // Syncread present position
//   dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
//   if (dxl_comm_result != COMM_SUCCESS)
//     Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

//   // Check if groupSyncRead data of Dynamixels are available
//   dxl_getdata_result = groupSyncReadEncoder_->isAvailable(left_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
//   if (dxl_getdata_result != true)
//     return false;

//   dxl_getdata_result = groupSyncReadEncoder_->isAvailable(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
//   if (dxl_getdata_result != true)
//     return false;

//   // Get data
//   left_value  = groupSyncReadEncoder_->getData(left_wheel_id_,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
//   right_value = groupSyncReadEncoder_->getData(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

//   groupSyncReadEncoder_->clearParam();
//   return true;
// }


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////




bool RiseMotorDriver::read_table_(
    int32_t &left_value, int32_t &right_value,
    dynamixel::GroupSyncRead *groupSync_, const uint16_t address, const uint16_t data_length)
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  // Set parameter
  dxl_addparam_result = groupSync_->addParam(left_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSync_->addParam(right_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  // Syncread present position
  dxl_comm_result = groupSync_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

  // Check if groupSyncRead data of Dynamixels are available
  dxl_getdata_result = groupSync_->isAvailable(left_wheel_id_, address, data_length);
  if (dxl_getdata_result != true)
    return false;

  dxl_getdata_result = groupSync_->isAvailable(right_wheel_id_, address, data_length);
  if (dxl_getdata_result != true)
    return false;

  // Get data
  // uint32_t    getData     (uint8_t id, uint16_t address, uint16_t data_length);
  left_value  = groupSync_->getData(left_wheel_id_,  address, data_length);
  right_value = groupSync_->getData(right_wheel_id_, address, data_length);

  groupSync_->clearParam();
  return true;
}


bool RiseMotorDriver::readEncoder(int32_t &left_value, int32_t &right_value)
{
  int32_t _left  = 0;
  int32_t _right = 0;
  bool _result = read_table_(_left, _right, groupSyncReadEncoder_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (_result)
  {
    left_value  = (int32_t)_left;
    right_value = (int32_t)_right;
  }
  return _result;
}

bool RiseMotorDriver::readPresentVel(int32_t &left_value, int32_t &right_value)
{
  int32_t _left  = 0;
  int32_t _right = 0;
  bool _result = read_table_(_left, _right, groupSyncReadPresentVel_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
  if (_result)
  {
    left_value  = (int32_t)_left;   // [0.229rev/min]
    right_value = (int32_t)_right;  // [0.229rev/min]
    // (1  / VELOCITY_CONSTANT_VALUE) = wheel_radius 0.033m * { 0.229[rev/min]*2pi[rad/rev]*(1/60)[min/sec] }
    // left_value  = ((float)_left) / VELOCITY_CONSTANT_VALUE;  // [m/s]
    // right_value = ((float)_right) / VELOCITY_CONSTANT_VALUE; // [m/s]
  }
  return _result;
}

bool RiseMotorDriver::readPresentPWM(int16_t &left_value, int16_t &right_value)
{
  int32_t _left  = 0;
  int32_t _right = 0;
  bool _result = read_table_(_left, _right, groupSyncReadPresentPWM_, 124, 2);
  if (_result)
  {
    left_value  = ((int16_t)_left);  // / 885.0;
    right_value = ((int16_t)_right); // / 885.0;
  }
  return _result;
}














// bool RiseMotorDriver::readHwError(uint8_t &left_value, uint8_t &right_value)
// {
//   int32_t _left  = 0;
//   int32_t _right = 0;
//   bool _result = read_table_(_left, _right, groupSyncReadHwError_, 70, 1);
//   if (_result)
//   {
//     left_value  = (uint8_t)_left;
//     right_value = (uint8_t)_right;
//   }
//   return _result;
// }
// bool RiseMotorDriver::readGoalPWM(float &left_value, float &right_value)
// {
//   int32_t _left  = 0;
//   int32_t _right = 0;
//   bool _result = read_table_(_left, _right, groupSyncReadGoalPWM_, 100, 2);
//   if (_result)
//   {
//     left_value  = ((float)_left) / 885.0;
//     right_value = ((float)_right) / 885.0;
//   }
//   return _result;
// }
// bool RiseMotorDriver::readGoalVel(float &left_value, float &right_value)
// {
//   int32_t _left  = 0;
//   int32_t _right = 0;
//   bool _result = read_table_(_left, _right, groupSyncReadGoalVel_, 104, 4);
//   if (_result)
//   {
//     left_value  = ((float)_left) / VELOCITY_CONSTANT_VALUE;
//     right_value = ((float)_right) / VELOCITY_CONSTANT_VALUE;
//   }
//   return _result;
// }











bool RiseMotorDriver::writeVelocity(int64_t left_value, int64_t right_value)
{
  bool dxl_addparam_result;
  int8_t dxl_comm_result;

  int64_t value[2] = {left_value, right_value};
  uint8_t data_byte[4] = {0, };

  for (uint8_t index = 0; index < 2; index++)
  {
    data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[index]));
    data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[index]));
    data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value[index]));
    data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value[index]));

    dxl_addparam_result = groupSyncWriteVelocity_->addParam(index+1, (uint8_t*)&data_byte);
    if (dxl_addparam_result != true)
      return false;
  }

  dxl_comm_result = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  groupSyncWriteVelocity_->clearParam();
  return true;
}

// bool RiseMotorDriver::writeWheelVelocity(float left_value, float right_value)
// {
//   bool dxl_addparam_result;
//   int8_t dxl_comm_result;

//   int64_t value[2] = {
//     (int64_t)constrain(left_value  * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY),
//     (int64_t)constrain(right_value * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY)};
//   uint8_t data_byte[4] = {0, };

//   for (uint8_t index = 0; index < 2; index++)
//   {
//     data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[index]));
//     data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[index]));
//     data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value[index]));
//     data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value[index]));

//     dxl_addparam_result = groupSyncWriteVelocity_->addParam(index+1, (uint8_t*)&data_byte);
//     if (dxl_addparam_result != true)
//       return false;
//   }

//   dxl_comm_result = groupSyncWriteVelocity_->txPacket();
//   if (dxl_comm_result != COMM_SUCCESS)
//   {
//     Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
//     return false;
//   }

//   groupSyncWriteVelocity_->clearParam();
//   return true;
// }

bool RiseMotorDriver::writePWM(float* topic_value)
{
  bool dxl_addparam_result;
  int8_t dxl_comm_result;

  // left
  int64_t value[2] = {
      (int64_t)constrain(topic_value[LEFT] * 885, -885, 885),
      (int64_t)constrain(topic_value[RIGHT] * 885, -885, 885)};
  uint8_t data_byte[2] = {0, };

  for (uint8_t index = 0; index < 2; index++)
  {
    data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[index]));
    data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[index]));

    dxl_addparam_result = groupSyncWritePWM_->addParam(index+1, (uint8_t*)&data_byte);
    if (dxl_addparam_result != true)
      return false;
  }

  dxl_comm_result = groupSyncWritePWM_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  groupSyncWritePWM_->clearParam();
  return true;
}














bool RiseMotorDriver::controlMotor(const float wheel_separation, float* value)
{
  bool dxl_comm_result = false;
  
  float wheel_velocity_cmd[2];

  float lin_vel = value[LEFT];
  float ang_vel = value[RIGHT];

  wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  // wheel_velocity_cmd[LEFT]   = - (ang_vel * wheel_separation / 2);
  // wheel_velocity_cmd[RIGHT]  = + (ang_vel * wheel_separation / 2);


  wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);

  dxl_comm_result = writeVelocity((int64_t)wheel_velocity_cmd[LEFT], (int64_t)wheel_velocity_cmd[RIGHT]);
  if (dxl_comm_result == false)
    return false;

  return true;
}
