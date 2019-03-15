/* These functions is used to send or read goal position and velocity 
   or present positions and velocities. There is a function that checks
   if the motor is still moving and the first functions enables/disables 
   the motor with ID. 
   
   created 12/10/2018
   script by erick nunez
*/

byte dxlAbling(byte mode, byte state){
  commResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER, ADDRESS_LED, state, &dxl_error);
  commResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW, ADDRESS_LED, state, &dxl_error);
  if (commResult){
    commResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER, ADDRESS_OPERATING_MODE, mode, &dxl_error);
    commResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW, ADDRESS_OPERATING_MODE, mode, &dxl_error);
  }
  if (commResult){
    commResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER, ADDRESS_TORQUE_ENABLE, !state, &dxl_error);
    commResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW, ADDRESS_TORQUE_ENABLE, !state, &dxl_error);
  } 
  return commResult;   
}

bool writeGoalPacket(int32_t goalVelElbow, int32_t goalPosElbow, int32_t goalVelShoulder, int32_t goalPosShoulder){
  // Elbow Parameters Goal Packet
  elbowParam[0] = DXL_LOBYTE(DXL_LOWORD(goalVelElbow));
  elbowParam[1] = DXL_HIBYTE(DXL_LOWORD(goalVelElbow));
  elbowParam[2] = DXL_LOBYTE(DXL_HIWORD(goalVelElbow));
  elbowParam[3] = DXL_HIBYTE(DXL_HIWORD(goalVelElbow));
  elbowParam[4] = DXL_LOBYTE(DXL_LOWORD(goalPosElbow));
  elbowParam[5] = DXL_HIBYTE(DXL_LOWORD(goalPosElbow));
  elbowParam[6] = DXL_LOBYTE(DXL_HIWORD(goalPosElbow));
  elbowParam[7] = DXL_HIBYTE(DXL_HIWORD(goalPosElbow));
  // Shoulder Parameters Goal Packet 
  shoulderParam[0] = DXL_LOBYTE(DXL_LOWORD(goalVelShoulder));
  shoulderParam[1] = DXL_HIBYTE(DXL_LOWORD(goalVelShoulder));
  shoulderParam[2] = DXL_LOBYTE(DXL_HIWORD(goalVelShoulder));
  shoulderParam[3] = DXL_HIBYTE(DXL_HIWORD(goalVelShoulder));
  shoulderParam[4] = DXL_LOBYTE(DXL_LOWORD(goalPosShoulder));
  shoulderParam[5] = DXL_HIBYTE(DXL_LOWORD(goalPosShoulder));
  shoulderParam[6] = DXL_LOBYTE(DXL_HIWORD(goalPosShoulder));
  shoulderParam[7] = DXL_HIBYTE(DXL_HIWORD(goalPosShoulder));
  // Writes packet
  addParamResult = syncWritePacket.addParam(ID_SHOULDER, shoulderParam);
  addParamResult = syncWritePacket.addParam(ID_ELBOW, elbowParam);
  commResult = syncWritePacket.txPacket();
  syncWritePacket.clearParam();
  return commResult;
}

void readPresentPacket(){
   commResult = syncReadPacket.txRxPacket();
   presVelElbow = syncReadPacket.getData(ID_ELBOW, ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
   presPosElbow = syncReadPacket.getData(ID_ELBOW, ADDRESS_PRESENT_POSITION, LEN_PRESENT_POSITION);
   presVelShoulder = syncReadPacket.getData(ID_SHOULDER, ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
   presPosShoulder = syncReadPacket.getData(ID_SHOULDER, ADDRESS_PRESENT_POSITION, LEN_PRESENT_POSITION);
}

byte isDxlMoving (int ID){
  //return dxl.readByte(ID,MOVING);
}
