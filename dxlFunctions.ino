/* These functions is used to send or read goal position and velocity 
   or present positions and velocities. There is a function that checks
   if the motor is still moving and the first functions enables/disables 
   the motor with ID. 
   
   created 12/10/2018
   script by erick nunez
*/

byte dxlEnable(int mode, boolean status){
  byte returnE = dxl.writeByte(ID_ELBOW,    OPERATING_MODE, mode);
  byte returnS = dxl.writeByte(ID_SHOULDER, OPERATING_MODE, mode);
  if (returnE & returnS){
    returnE = dxl.writeByte(ID_ELBOW,    TORQUE_ENABLE,  status);
    returnS = dxl.writeByte(ID_SHOULDER, TORQUE_ENABLE,  status);
  }
  if (returnE & returnS){
    returnE = dxl.writeByte(ID_ELBOW,    LED,            status);
    returnS = dxl.writeByte(ID_SHOULDER, LED,            status);
  } 
  return returnE & returnS;   
}

byte dxlGoalVelPos(int goalVelElbow, int goalPosElbow, int goalVelShoulder, int goalPosShoulder){
  byte velReturnE = dxl.writeDword(ID_ELBOW,    PROFILE_VELOCITY, goalVelElbow);
  //byte velReturnS = dxl.writeDword(ID_SHOULDER, PROFILE_VELOCITY, goalVelShoulder);
  byte posReturnE = dxl.writeDword(ID_ELBOW,    GOAL_POSITION,    goalPosElbow);
  //byte posReturnS = dxl.writeDword(ID_SHOULDER, GOAL_POSITION,    goalPosShoulder);
  return velReturnE & posReturnE;
}

void dxlPresVelPos(){
   presVelElbow    = (int32)dxl.readDword(ID_ELBOW,    PRESENT_VELOCITY);
   presVelShoulder = (int32)dxl.readDword(ID_SHOULDER, PRESENT_VELOCITY);
   presPosElbow    = (int32)dxl.readDword(ID_ELBOW,    PRESENT_POSITION);
   presPosShoulder = (int32)dxl.readDword(ID_SHOULDER, PRESENT_POSITION);
}

byte isDxlMoving (int ID){
  return dxl.readByte(ID,MOVING);
}
