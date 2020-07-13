/* These functions is used to send or read goal position and velocity
   or present positions and velocities. There is a function that checks
   if the motor is still moving and the first functions enables/disables
   the motor with ID.

   created 12/10/2018
   script by erick nunez
*/

void dxlAbling(uint8_t mode, uint8_t state, uint8_t &dxl_error) {
  int dxlCommResult;
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_LED, state, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_LED, state, &dxl_error);
  
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_OPERATING_MODE, mode, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER_SLV, ADDRESS_OPERATING_MODE, mode, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_OPERATING_MODE, mode, &dxl_error);
  
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_VELOCITY_LIMIT, VELOCITY_LIMIT,   &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_VELOCITY_LIMIT, VELOCITY_LIMIT,   &dxl_error);
//  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_MIN_POSITION,   SHOULDER_MIN_POS, &dxl_error);
//  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_MAX_POSITION,   SHOULDER_MAX_POS, &dxl_error);
//  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_MIN_POSITION,   ELBOW_MIN_POS,    &dxl_error);
//  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_MAX_POSITION,   ELBOW_MAX_POS,    &dxl_error);
  
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_TORQUE_ENABLE, state, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_TORQUE_ENABLE, state, &dxl_error);

  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_PROFILE_VELOCITY, ~state, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_PROFILE_VELOCITY, ~state, &dxl_error);
}

int writeGoalPacket(bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket, jointSpace goal) {
  int dxlCommResult;
  //uint8_t elbowParam[8], shoulderParam[8];
  uint8_t elbowParam[4], shoulderParam[4];
  
  // Convert to Counts /////////////////////////////////////////////////////////////////////////
  int32_t goalPosShoulder = goal.q1 * (180.0 / PI) / DEGREES_PER_COUNT;
  int32_t goalPosElbow    = ELBOW_MIN_POS + goal.q4 * (180.0 / PI) / DEGREES_PER_COUNT;
  int32_t goalVelShoulder = abs(goal.q1Dot * (60.0 / (2.0 * PI)) / RPM_PER_COUNT);
  int32_t goalVelElbow    = abs(goal.q4Dot    * (60.0 / (2.0 * PI)) / RPM_PER_COUNT);
  
  // Elbow Parameters Goal Packet /////////////////////////////////////////////////////////////////////////
  //elbowParam[0] = DXL_LOBYTE(DXL_LOWORD(goalVelElbow));
  //elbowParam[1] = DXL_HIBYTE(DXL_LOWORD(goalVelElbow));
  //elbowParam[2] = DXL_LOBYTE(DXL_HIWORD(goalVelElbow));
  //elbowParam[3] = DXL_HIBYTE(DXL_HIWORD(goalVelElbow));
  //elbowParam[4] = DXL_LOBYTE(DXL_LOWORD(goalPosElbow));
  //elbowParam[5] = DXL_HIBYTE(DXL_LOWORD(goalPosElbow));
  //elbowParam[6] = DXL_LOBYTE(DXL_HIWORD(goalPosElbow));
  //elbowParam[7] = DXL_HIBYTE(DXL_HIWORD(goalPosElbow));
  elbowParam[0] = DXL_LOBYTE(DXL_LOWORD(goalPosElbow));
  elbowParam[1] = DXL_HIBYTE(DXL_LOWORD(goalPosElbow));
  elbowParam[2] = DXL_LOBYTE(DXL_HIWORD(goalPosElbow));
  elbowParam[3] = DXL_HIBYTE(DXL_HIWORD(goalPosElbow));
  
  // Shoulder Parameters Goal Packet /////////////////////////////////////////////////////////////////////////
  //shoulderParam[0] = DXL_LOBYTE(DXL_LOWORD(goalVelShoulder));
  //shoulderParam[1] = DXL_HIBYTE(DXL_LOWORD(goalVelShoulder));
  //shoulderParam[2] = DXL_LOBYTE(DXL_HIWORD(goalVelShoulder));
  //shoulderParam[3] = DXL_HIBYTE(DXL_HIWORD(goalVelShoulder));
  //shoulderParam[4] = DXL_LOBYTE(DXL_LOWORD(goalPosShoulder));
  //shoulderParam[5] = DXL_HIBYTE(DXL_LOWORD(goalPosShoulder));
  //shoulderParam[6] = DXL_LOBYTE(DXL_HIWORD(goalPosShoulder));
  //shoulderParam[7] = DXL_HIBYTE(DXL_HIWORD(goalPosShoulder));
  shoulderParam[0] = DXL_LOBYTE(DXL_LOWORD(goalPosShoulder));
  shoulderParam[1] = DXL_HIBYTE(DXL_LOWORD(goalPosShoulder));
  shoulderParam[2] = DXL_LOBYTE(DXL_HIWORD(goalPosShoulder));
  shoulderParam[3] = DXL_HIBYTE(DXL_HIWORD(goalPosShoulder));
  
  // Writes packet /////////////////////////////////////////////////////////////////////////
  addParamResult = syncWritePacket.addParam(ID_SHOULDER, shoulderParam);
  addParamResult = syncWritePacket.addParam(ID_ELBOW, elbowParam);
  dxlCommResult = syncWritePacket.txPacket();
  syncWritePacket.clearParam();
  return dxlCommResult;
}

jointSpace readPresentPacket(dynamixel::GroupSyncRead  &syncReadPacket) {
  jointSpace pres;
  
  /* Read Position and Velocity */
  int dxlCommResult = syncReadPacket.txRxPacket();
  int32_t presVelElbow = -syncReadPacket.getData(ID_ELBOW, ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY); // Elbow Direction appears to be inversed.
  int32_t presPosElbow = syncReadPacket.getData(ID_ELBOW, ADDRESS_PRESENT_POSITION, LEN_PRESENT_POSITION);
  int32_t presVelShoulder = syncReadPacket.getData(ID_SHOULDER, ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
  int32_t presPosShoulder = syncReadPacket.getData(ID_SHOULDER, ADDRESS_PRESENT_POSITION, LEN_PRESENT_POSITION);
  
  /* Convert Motor Counts */
  pres.q1      = (presPosShoulder) * DEGREES_PER_COUNT * (PI / 180.0);
  pres.q4      = (presPosElbow - ELBOW_MIN_POS) * DEGREES_PER_COUNT * (PI / 180.0);
  pres.q1Dot   = presVelShoulder * RPM_PER_COUNT * (2.0 * PI / 60.0);
  pres.q4Dot   = presVelElbow    * RPM_PER_COUNT * (2.0 * PI / 60.0);

  /* Elevation */
  pres.q2 = -(ELEVATION_ZERO - analogRead(ELEVATION_SENSOR_PIN)*((2*PI)/1023));
  pres.q2Dot = 0;

  return pres;
}
