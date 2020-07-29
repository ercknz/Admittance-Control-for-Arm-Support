/* These functions is used to send or read goal position and velocity
   or present positions and velocities. There is a function that checks
   if the motor is still moving and the first functions enables/disables
   the motor with ID.

   created 12/10/2018
   script by erick nunez
*/

void dxlConfig(uint8_t &dxl_error) {
  int dxlCommResult;
  /* Enable LED for visual indication */
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_LED, ENABLE, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_LED, ENABLE, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHLDR_ELEVATE,ADDRESS_LED, ENABLE, &dxl_error);
  /* Sets control mode */
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_OPERATING_MODE, POSITION_CONTROL, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHLDR_ELEVATE,ADDRESS_OPERATING_MODE, POSITION_CONTROL, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_OPERATING_MODE, POSITION_CONTROL, &dxl_error);
  /* Set Velocity Limits */
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_VELOCITY_LIMIT, VELOCITY_LIMIT,   &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHLDR_ELEVATE,ADDRESS_VELOCITY_LIMIT, VELOCITY_LIMIT,   &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_VELOCITY_LIMIT, VELOCITY_LIMIT,   &dxl_error);
  /* Sets Position Limits */
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_MIN_POSITION,   SHOULDER_MIN_POS, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_MAX_POSITION,   SHOULDER_MAX_POS, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHLDR_ELEVATE,ADDRESS_MIN_POSITION,   ELEVATION_MIN_POS,&dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHLDR_ELEVATE,ADDRESS_MAX_POSITION,   ELEVATION_MAX_POS,&dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_MIN_POSITION,   ELBOW_MIN_POS,    &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_MAX_POSITION,   ELBOW_MAX_POS,    &dxl_error);
  /* Sets Velocity Profiles */
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_PROFILE_VELOCITY, VEL_BASED_PROFILE, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHLDR_ELEVATE,ADDRESS_PROFILE_VELOCITY, VEL_BASED_PROFILE, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_PROFILE_VELOCITY, VEL_BASED_PROFILE, &dxl_error);
}

void dxlTorque(bool state, uint8_t &dxl_error) {
  int dxlCommResult;
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_TORQUE_ENABLE, state, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHLDR_ELEVATE,ADDRESS_TORQUE_ENABLE, state, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_TORQUE_ENABLE, state, &dxl_error);
}

int writeGoalPacket(bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket, jointSpace goal) {
  int dxlCommResult;
  //uint8_t elbowParam[8], shoulderParam[8], elevateParam[8];
  uint8_t elbowParam[4], shoulderParam[4], elevateParam[4];
  
  /* Elbow Parameters Goal Packet */
  elbowParam[0] = DXL_LOBYTE(DXL_LOWORD(goal.q4Cts));
  elbowParam[1] = DXL_HIBYTE(DXL_LOWORD(goal.q4Cts));
  elbowParam[2] = DXL_LOBYTE(DXL_HIWORD(goal.q4Cts));
  elbowParam[3] = DXL_HIBYTE(DXL_HIWORD(goal.q4Cts));
  
  /* Shoulder Parameters Goal Packet */
  shoulderParam[0] = DXL_LOBYTE(DXL_LOWORD(goal.q1Cts));
  shoulderParam[1] = DXL_HIBYTE(DXL_LOWORD(goal.q1Cts));
  shoulderParam[2] = DXL_LOBYTE(DXL_HIWORD(goal.q1Cts));
  shoulderParam[3] = DXL_HIBYTE(DXL_HIWORD(goal.q1Cts));

  /* Elevation Parameters Goal Packet */
  elevateParam[0] = DXL_LOBYTE(DXL_LOWORD(goal.q2Cts));
  elevateParam[1] = DXL_HIBYTE(DXL_LOWORD(goal.q2Cts));
  elevateParam[2] = DXL_LOBYTE(DXL_HIWORD(goal.q2Cts));
  elevateParam[3] = DXL_HIBYTE(DXL_HIWORD(goal.q2Cts));
  
  /* Writes packet */
  addParamResult = syncWritePacket.addParam(ID_SHOULDER,      shoulderParam);
  addParamResult = syncWritePacket.addParam(ID_SHLDR_ELEVATE, elevateParam);
  addParamResult = syncWritePacket.addParam(ID_ELBOW,         elbowParam);
  dxlCommResult = syncWritePacket.txPacket();
  syncWritePacket.clearParam();
  
  return dxlCommResult;
}

jointSpace readPresentPacket(dynamixel::GroupSyncRead  &syncReadPacket) {
  jointSpace pres;
  
  /* Read Position and Velocity */
  int dxlCommResult = syncReadPacket.txRxPacket();
  pres.q4DotCts = syncReadPacket.getData(ID_ELBOW,         ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY); // Elbow Direction appears to be inversed.
  pres.q4Cts    = syncReadPacket.getData(ID_ELBOW,         ADDRESS_PRESENT_POSITION, LEN_PRESENT_POSITION);
  pres.q2DotCts = syncReadPacket.getData(ID_SHLDR_ELEVATE, ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
  pres.q2Cts    = syncReadPacket.getData(ID_SHLDR_ELEVATE, ADDRESS_PRESENT_POSITION, LEN_PRESENT_POSITION);
  pres.q1DotCts = syncReadPacket.getData(ID_SHOULDER,      ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
  pres.q1Cts    = syncReadPacket.getData(ID_SHOULDER,      ADDRESS_PRESENT_POSITION, LEN_PRESENT_POSITION);
  
  /* Convert Motor Counts */
  pres.q1      = (pres.q1Cts) * DEGREES_PER_COUNT * (PI / 180.0);
  pres.q2      = (pres.q2Cts) * DEGREES_PER_COUNT * (PI / 180.0);
  pres.q4      = (pres.q4Cts) * DEGREES_PER_COUNT * (PI / 180.0); // - ELBOW_MIN_POS
  pres.q1Dot   = pres.q1DotCts * RPM_PER_COUNT * (2.0 * PI / 60.0);
  pres.q2Dot   = pres.q2DotCts * RPM_PER_COUNT * (2.0 * PI / 60.0);
  pres.q4Dot   = pres.q4DotCts * RPM_PER_COUNT * (2.0 * PI / 60.0);

  return pres;
}
