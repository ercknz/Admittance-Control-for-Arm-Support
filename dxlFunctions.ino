/* These functions is used to send or read goal position and velocity
   or present positions and velocities. There is a function that checks
   if the motor is still moving and the first functions enables/disables
   the motor with ID.

   created 12/10/2018
   script by erick nunez
*/

void dxlAbling(uint8_t mode, uint8_t state, uint8_t &dxl_error) {
  int dxlCommResult;
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER, ADDRESS_LED, state, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW, ADDRESS_LED, state, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER, ADDRESS_OPERATING_MODE, mode, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW, ADDRESS_OPERATING_MODE, mode, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER, ADDRESS_TORQUE_ENABLE, state, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW, ADDRESS_TORQUE_ENABLE, state, &dxl_error);
}

int writeGoalPacket(bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket, int32_t goalVelElbow, int32_t goalPosElbow, int32_t goalVelShoulder, int32_t goalPosShoulder) {
  int dxlCommResult;
  uint8_t elbowParam[8], shoulderParam[8];
  // Check limits before writing /////////////////////////////////////////////////////////////////////////
  if (goalVelElbow > ELBOW_MAX_VEL) {
    goalVelElbow = ELBOW_MAX_VEL;
  } else if (goalVelElbow < ELBOW_MIN_VEL) {
    goalVelElbow = ELBOW_MIN_VEL;
  }
  if (goalVelShoulder > SHOULDER_MAX_VEL) {
    goalVelShoulder = SHOULDER_MAX_VEL;
  } else if (goalVelShoulder < SHOULDER_MIN_VEL) {
    goalVelShoulder = SHOULDER_MIN_VEL;
  }
  if ((goalPosElbow < ELBOW_MAX_POS) && (goalPosElbow > ELBOW_MIN_POS)) {
    if ((goalPosShoulder < SHOULDER_MAX_POS) && (goalPosShoulder > SHOULDER_MIN_POS)) {
      // Elbow Parameters Goal Packet /////////////////////////////////////////////////////////////////////////
      elbowParam[0] = DXL_LOBYTE(DXL_LOWORD(goalVelElbow));
      elbowParam[1] = DXL_HIBYTE(DXL_LOWORD(goalVelElbow));
      elbowParam[2] = DXL_LOBYTE(DXL_HIWORD(goalVelElbow));
      elbowParam[3] = DXL_HIBYTE(DXL_HIWORD(goalVelElbow));
      elbowParam[4] = DXL_LOBYTE(DXL_LOWORD(goalPosElbow));
      elbowParam[5] = DXL_HIBYTE(DXL_LOWORD(goalPosElbow));
      elbowParam[6] = DXL_LOBYTE(DXL_HIWORD(goalPosElbow));
      elbowParam[7] = DXL_HIBYTE(DXL_HIWORD(goalPosElbow));
      // Shoulder Parameters Goal Packet /////////////////////////////////////////////////////////////////////////
      shoulderParam[0] = DXL_LOBYTE(DXL_LOWORD(goalVelShoulder));
      shoulderParam[1] = DXL_HIBYTE(DXL_LOWORD(goalVelShoulder));
      shoulderParam[2] = DXL_LOBYTE(DXL_HIWORD(goalVelShoulder));
      shoulderParam[3] = DXL_HIBYTE(DXL_HIWORD(goalVelShoulder));
      shoulderParam[4] = DXL_LOBYTE(DXL_LOWORD(goalPosShoulder));
      shoulderParam[5] = DXL_HIBYTE(DXL_LOWORD(goalPosShoulder));
      shoulderParam[6] = DXL_LOBYTE(DXL_HIWORD(goalPosShoulder));
      shoulderParam[7] = DXL_HIBYTE(DXL_HIWORD(goalPosShoulder));
      // Writes packet /////////////////////////////////////////////////////////////////////////
      addParamResult = syncWritePacket.addParam(ID_SHOULDER, shoulderParam);
      addParamResult = syncWritePacket.addParam(ID_ELBOW, elbowParam);
      dxlCommResult = syncWritePacket.txPacket();
      syncWritePacket.clearParam();
      return dxlCommResult;
    }
  }
}

int readPresentPacket(dynamixel::GroupSyncRead  &syncReadPacket, int32_t &presVelElbow, int32_t &presPosElbow, int32_t &presVelShoulder, int32_t &presPosShoulder) {
  int dxlCommResult = syncReadPacket.txRxPacket();
  presVelElbow = -syncReadPacket.getData(ID_ELBOW, ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY); // Elbow Direction appears to be inversed.
  presPosElbow = syncReadPacket.getData(ID_ELBOW, ADDRESS_PRESENT_POSITION, LEN_PRESENT_POSITION);
  presVelShoulder = syncReadPacket.getData(ID_SHOULDER, ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
  presPosShoulder = syncReadPacket.getData(ID_SHOULDER, ADDRESS_PRESENT_POSITION, LEN_PRESENT_POSITION);
  return dxlCommResult;
}
