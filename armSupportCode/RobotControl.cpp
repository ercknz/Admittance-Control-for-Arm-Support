/* Class controls the arm support robot
   It includes the fKine and iKine as well as the
   functions needed to write and read to the motors
   Refer to github-Dynamixel for more information on dynamixel library.

   Class arrays use the following:
   xyz[3]     = {x, y, x}; 
   xyzDot[3]  = {xDot, yDot, zDot};
   q[3]       = {q1(shoulder), q2(elevation), q4(elbow)};
   qDot[3]    = {q1Dot(shoulder), q2Dot(elevation), q4Dot(elbow)}

   Created 10/28/2020
   Script by Erick Nunez
*/

#include <DynamixelSDK.h>
#include <math.h>
#include "RobotControl.h"
#include "armSupportNamespace.h"

/******************** Arm Support Constructor  ***********************************************************************/
RobotControl::RobotControl(const float A1, const float L1, const float A2, const float L2, const float Offset)
  :_A1A2{A1 + A2},
  _L1{L1},
  _L2{L2},
  _OFFSET{Offset},
  _PHI{atan(Offset / L2)},
  _H_OF_L2{sqrt(pow(Offset, 2) + pow(L2, 2))},
  _Q1_MIN{ASR::SHOULDER_MIN_POS * ASR::DEGREES_PER_COUNT * (PI / 180.0)},
  _Q1_MAX{ASR::SHOULDER_MAX_POS * ASR::DEGREES_PER_COUNT * (PI / 180.0)},
  _Q2_LIMIT{(ASR::ELEVATION_MAX_POS - ASR::ELEVATION_CENTER) * ASR::DEGREES_PER_COUNT * (PI / 180.0) * (1 / ASR::ELEVATION_RATIO)},
  _Q4_MIN{(ASR::ELBOW_MIN_POS - ASR::ELBOW_MIN_POS) * ASR::DEGREES_PER_COUNT * (PI / 180.0)},
  _Q4_MAX{(ASR::ELBOW_MAX_POS - ASR::ELBOW_MIN_POS) * ASR::DEGREES_PER_COUNT * (PI / 180.0)},
  _INNER_R{A1 + L1 + A2 - L2},
  _Z_LIMIT{L1 * sin((ASR::ELEVATION_MAX_POS - ASR::ELEVATION_CENTER) * ASR::DEGREES_PER_COUNT * (PI / 180.0) * (1 / ASR::ELEVATION_RATIO))}
{
}

/******************** Arm Support Get Member functions  ***********************************************************************/
float* RobotControl::GetPresQCts(){ 
  return qPresCts_M;
}

float* RobotControl::GetPresQDotCts(){
  return qDotPresCts_M;
}

float* RobotControl::GetPresQ(){
  return qPres_M;
}

float* RobotControl::GetPresQDot(){
  return qDotPres_M;
}

float* RobotControl::GetPresPos(){ 
  return xyzPres_M;
}

float* RobotControl::GetPresVel(){
  return xyzDotPres_M;
}

float* RobotControl::GetGoalQCts(){ 
  return qCts_M;
}

float* RobotControl::GetGoalQDotCts(){
  return qDotCts_M;
}

float* RobotControl::GetGoalQ(){
  return q_M;
}

float* RobotControl::GetGoalQDot(){
  return qDot_M;
}

/******************** Arm Support Motors Reading/Writing  ***********************************************************************/
void RobotControl::ReadRobot(dynamixel::GroupSyncRead &syncReadPacket){
  ReadMotors(syncReadPacket);
  fKine();
}

void RobotControl::WriteToRobot(float xyz, float xyzDot, bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket){
  iKine(xyz, xyzDot);
  int returnInt = WriteToMotors(addParamResult, syncWritePacket);
}

/******************** Arm Support Inverse Kinematics Member function ************************************************/
void RobotControl::iKine(float &xyz, float &xyzDot) {
  float L1_XY, OUTER_R, R, alpha, beta, gamma, detJ;
  for (int i=0; i<3; i++){
    xyz_M[i]    +=  xyz[i];
    xyzDot_M[i] = xyzDot[i];
  }

  /* Check Z limits */
  if (xyz_M[2] >  _Z_LIMIT) xyz_M[2] =  _Z_LIMIT;
  if (xyz_M[2] < -_Z_LIMIT) xyz_M[2] = -_Z_LIMIT;

  /* Find variables based on Z */
  q_M[1]  = asin(xyz_M[2] / _L1);
  L1_XY   = sqrt(pow(_L1, 2) - pow(xyz_M[2], 2));

   /* Checks if X and Y are both 0 */
  if ((abs(xyz[0]) < 0.001) && (abs(xyz[1]) < 0.001)) {
    q_M[0] = qPres_M[0];
    q_M[2] = _Q4_MAX;
    xyz_M[0] = _A1A2 * cos(q_M[0]) + _L1 * cos(q_M[0]) * cos(q_M[1]) + _OFFSET * sin(q_M[0] + q_M[2]) + _L2 * cos(q_M[0] + q_M[2]);
    xyz_M[1] = _A1A2 * sin(q_M[0]) + _L1 * sin(q_M[0]) * cos(q_M[1]) - _OFFSET * cos(q_M[0] + q_M[2]) + _L2 * sin(q_M[0] + q_M[2]);
  }

  /* Checks walls */
  OUTER_R = _A1A2 + _H_OF_L2 + L1_XY;
  R       = sqrt(pow(xyz_M[0], 2) + pow(xyz_M[1], 2));
  alpha   = atan2(xyz_M[1], xyz_M[0]);
  if (alpha < 0.0f) alpha += 2 * PI;
  if (R < _INNER_R) {
    xyz_M[0]  = _INNER_R * cos(alpha);
    xyz_M[1]  = _INNER_R * sin(alpha);
    R         = _INNER_R;
  }
  if (R > OUTER_R) {
    xyz_M[0]  = OUTER_R * cos(alpha);
    xyz_M[1]  = OUTER_R * sin(alpha);
    R         = OUTER_R;
  }

  /* Finds and checks Elbow Angle */
  gamma = acos((pow((_A1A2 + L1_XY), 2) + pow(_H_OF_L2, 2) - pow(xyz_M[0], 2) - pow(xyz_M[1], 2)) / (2 * _H_OF_L2 * (_A1A2 + L1_XY)));

  q_M[2] = PI - gamma;
  if (q_M[2] < _Q4_MIN) q_M[2] = _Q4_MIN;
  if (q_M[2] > _Q4_MAX) q_M[2] = _Q4_MAX;

  /* Finds and checks shoulder angle */
  beta = asin((_H_OF_L2 * sin(gamma)) / R);
  q_M[0] = alpha - beta;
  if (q_M[0] < _Q1_MIN) q_M[0] = _Q1_MIN;
  if (q_M[0] > _Q1_MAX) q_M[0] = _Q1_MAX;

  /* Check for nans */
  if (q_M[0] != q_M[0]) q_M[0] = qPres_M[0];
  if (q_M[2] != q_M[2]) q_M[2] = qPres_M[2];

  /* Checks XYZ */
  float xyzCheck[3];
  xyzCheck[0] = _A1A2 * cos(q_M[0]) + _L1 * cos(q_M[0]) * cos(q_M[1]) + _OFFSET * sin(q_M[0] + q_M[2]) + _L2 * cos(q_M[0] + q_M[2]);
  xyzCheck[1] = _A1A2 * sin(q_M[0]) + _L1 * sin(q_M[0]) * cos(q_M[1]) - _OFFSET * cos(q_M[0] + q_M[2]) + _L2 * sin(q_M[0] + q_M[2]);
  xyzCheck[2] =   _L1 * sin(q_M[1]);
//  if ((abs(M.x - checkM.x) > 0.001) || (abs(M.y - checkM.y) > 0.001) || (abs(M.y - checkM.y) > 0.001)) {
//    M = checkM;
//  }

  /* Solve for joint angular velocities (psuedo inverse Jacobian) */
  detJ      = (-_L1 * sin(q_M[0]) - _L2 * sin(q_M[0] + q_M[2])) * (_L2 * cos(q_M[0] + q_M[2])) - (-_L2 * sin(q_M[0] + q_M[2])) * (_L1 * cos(q_M[0]) + _L2 * cos(q_M[0] + q_M[2]));
  qDot_M[0] = (xyzDot_M[0] * (_L2 * cos(q_M[0] + q_M[2])) + xyzDot_M[1] * (_L2 * sin(q_M[0] + q_M[2]))) / detJ;
  qDot_M[1] = xyzDot_M[2] / (_L1 * sqrt(1 - pow((xyz_M[2] / _L1), 2)));
  qDot_M[2] = -(xyzDot_M[0] * (-_L1 * cos(q_M[0]) - _L2 * cos(q_M[0] + q_M[2])) + xyzDot_M[1] * (-_L1 * sin(q_M[0]) - _L2 * sin(q_M[0] + q_M[2]))) / detJ;
}

/******************** Arm Support Forward Kinematics Member Function ************************************************/
void  RobotControl::fKine() {
  // Compute the XY positions from angles
  xyzPres_M[0] = _A1A2 * cos(qPres_M[0]) + _L1 * cos(qPres_M[0]) * cos(qPres_M[1]) + _OFFSET * sin(qPres_M[0] + qPres_M[2]) + _L2 * cos(qPres_M[0] + qPres_M[2]);
  xyzPres_M[1] = _A1A2 * sin(qPres_M[0]) + _L1 * sin(qPres_M[0]) * cos(qPres_M[1]) - _OFFSET * cos(qPres_M[0] + qPres_M[2]) + _L2 * sin(qPres_M[0] + qPres_M[2]);
  xyzPres_M[2] =   _L1 * sin(qPres_M[1]);

  // Multiply velocities with Jacobian Matrix to find the XY velocities
  J_M[0][0] = - _A1A2   * sin(qPres_M[0]) - _L1 * sin(qPres_M[0]) * cos(qPres_M[1]) + _OFFSET * cos(qPres_M[0] + qPres_M[2]) - _L2 * sin(qPres_M[0] + qPres_M[2]);
  J_M[0][1] = - _L1     * cos(qPres_M[0]) * sin(qPres_M[1]);
  J_M[0][2] =   _OFFSET * cos(qPres_M[0] + qPres_M[2]) - _L2 * sin(qPres_M[0] + qPres_M[2]);
  J_M[1][0] =   _A1A2   * cos(qPres_M[0]) + _L1 * cos(qPres_M[0]) * cos(qPres_M[1]) + _OFFSET * sin(qPres_M[0] + qPres_M[2]) + _L2 * cos(qPres_M[0] + qPres_M[2]);
  J_M[1][1] = - _L1     * sin(qPres_M[0]) * sin(qPres_M[1]);
  J_M[1][2] =   _OFFSET * sin(qPres_M[0] + qPres_M[2]) + _L2 * cos(qPres_M[0] + qPres_M[2]);
  J_M[2][1] =   _L1     * cos(qPres_M[1]);  // J31 = J33 = 0.0
  
  xyzDotPres_M[0] = qDotPres_M[0] * J_M[0][0] + qDotPres_M[1] * J_M[0][1] + qDotPres_M[2] * J_M[0][2];
  xyzDotPres_M[1] = qDotPres_M[0] * J_M[1][0] + qDotPres_M[1] * J_M[1][1] + qDotPres_M[2] * J_M[1][2];
  xyzDotPres_M[2] = qDotPres_M[0] * J_M[2][0] + qDotPres_M[1] * J_M[2][1] + qDotPres_M[2] * J_M[2][2];
}

/******************** Arm Support DXL Torque Enabling Member Function ************************************************/
void  RobotControl::EnableTorque(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler  *packetHandler, uint8_t state) {
  using namespace ASR;
  int dxlCommResult;
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_TORQUE_ENABLE, state, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_TORQUE_ENABLE, state, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_TORQUE_ENABLE, state, &dxl_error);
}

/******************** Arm Support DXL Configuration Member Function ************************************************/
void  RobotControl::MotorConfig(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler  *packetHandler) {
  using namespace ASR;
  int dxlCommResult;
  /* Enable LED for visual indication */
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_LED, ENABLE, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_LED, ENABLE, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_LED, ENABLE, &dxl_error);
  /* Sets control mode */
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_OPERATING_MODE, POSITION_CONTROL, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_OPERATING_MODE, POSITION_CONTROL, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_OPERATING_MODE, POSITION_CONTROL, &dxl_error);
  /* Set Velocity Limits */
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_VELOCITY_LIMIT, VEL_MAX_LIMIT,   &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_VELOCITY_LIMIT, VEL_MAX_LIMIT,   &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_VELOCITY_LIMIT, VEL_MAX_LIMIT,   &dxl_error);
  /* Sets Position Limits */
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_MIN_POSITION,   SHOULDER_MIN_POS, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_MAX_POSITION,   SHOULDER_MAX_POS, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_MIN_POSITION,   ELEVATION_MIN_POS, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_MAX_POSITION,   ELEVATION_MAX_POS, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_MIN_POSITION,   ELBOW_MIN_POS,    &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_MAX_POSITION,   ELBOW_MAX_POS,    &dxl_error);
  /* Sets Velocity Profiles */
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_PROFILE_VELOCITY, VEL_BASED_PROFILE, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_PROFILE_VELOCITY, VEL_BASED_PROFILE, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_PROFILE_VELOCITY, VEL_BASED_PROFILE, &dxl_error);
}

/******************** Arm Support DXL Read Member Function ************************************************/
void  RobotControl::ReadMotors(dynamixel::GroupSyncRead  &syncReadPacket) {
  using namespace ASR;
  /* Read Position and Velocity */
  int dxlCommResult = syncReadPacket.txRxPacket();
  qPresCts_M[0]    = syncReadPacket.getData(ID_SHOULDER,     ADDRESS_PRESENT_POSITION, LEN_PRESENT_POSITION);
  qPresCts_M[1]    = syncReadPacket.getData(ID_ELEVATION,    ADDRESS_PRESENT_POSITION, LEN_PRESENT_POSITION);
  qPresCts_M[2]    = syncReadPacket.getData(ID_ELBOW,        ADDRESS_PRESENT_POSITION, LEN_PRESENT_POSITION);
  qDotPresCts_M[0] = syncReadPacket.getData(ID_SHOULDER,     ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
  qDotPresCts_M[1] = syncReadPacket.getData(ID_ELEVATION,    ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
  qDotPresCts_M[2] = syncReadPacket.getData(ID_ELBOW,        ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);

  /* Convert Motor Counts */
  qPres_M[0]      =  (qPresCts_M[0]) * DEGREES_PER_COUNT * (PI / 180.0);
  qPres_M[1]      = -(qPresCts_M[1] - ELEVATION_CENTER) * DEGREES_PER_COUNT * (PI / 180.0) * (1 / ELEVATION_RATIO);
  qPres_M[2]      =  (qPresCts_M[2] - ELBOW_MIN_POS) * DEGREES_PER_COUNT * (PI / 180.0);
  qDotPres_M[0]   = qDotPresCts_M[0] * RPM_PER_COUNT * (2.0 * PI / 60.0);
  qDotPres_M[1]   = qDotPresCts_M[1] * RPM_PER_COUNT * (2.0 * PI / 60.0) * (1 / ELEVATION_RATIO);
  qDotPres_M[2]   = qDotPresCts_M[2] * RPM_PER_COUNT * (2.0 * PI / 60.0);
}

/******************** Arm Support DXL Write Member Function ************************************************/
int  RobotControl::WriteToMotors(bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket) {
  using namespace ASR;
  /* Convert to Counts */
  qCts_M[0]    = q_M[0] * (180.0 / PI) / DEGREES_PER_COUNT;
  qCts_M[1]    = ELEVATION_CENTER - (q_M[1] * ELEVATION_RATIO * (180.0 / PI) / DEGREES_PER_COUNT);
  qCts_M[2]    = ELBOW_MIN_POS + q_M[2] * (180.0 / PI) / DEGREES_PER_COUNT;
  qDotCts_M[0] = abs(qDot_M[0] * (60.0 / (2.0 * PI)) / RPM_PER_COUNT);
  qDotCts_M[1] = abs(qDot_M[1] * (60.0 / (2.0 * PI)) / RPM_PER_COUNT) * ELEVATION_RATIO;
  qDotCts_M[2] = abs(qDot_M[2] * (60.0 / (2.0 * PI)) / RPM_PER_COUNT);
  
  int dxlCommResult;
  //uint8_t elbowParam[8], shoulderParam[8], elevateParam[8];
  uint8_t elbowParam[4], shoulderParam[4], elevateParam[4];

  /* Shoulder Parameters Goal Packet */
  shoulderParam[0] = DXL_LOBYTE(DXL_LOWORD(qCts_M[0]));
  shoulderParam[1] = DXL_HIBYTE(DXL_LOWORD(qCts_M[0]));
  shoulderParam[2] = DXL_LOBYTE(DXL_HIWORD(qCts_M[0]));
  shoulderParam[3] = DXL_HIBYTE(DXL_HIWORD(qCts_M[0]));

  /* Elevation Parameters Goal Packet */
  elevateParam[0] = DXL_LOBYTE(DXL_LOWORD(qCts_M[1]));
  elevateParam[1] = DXL_HIBYTE(DXL_LOWORD(qCts_M[1]));
  elevateParam[2] = DXL_LOBYTE(DXL_HIWORD(qCts_M[1]));
  elevateParam[3] = DXL_HIBYTE(DXL_HIWORD(qCts_M[1]));

  /* Elbow Parameters Goal Packet */
  elbowParam[0] = DXL_LOBYTE(DXL_LOWORD(qCts_M[2]));
  elbowParam[1] = DXL_HIBYTE(DXL_LOWORD(qCts_M[2]));
  elbowParam[2] = DXL_LOBYTE(DXL_HIWORD(qCts_M[2]));
  elbowParam[3] = DXL_HIBYTE(DXL_HIWORD(qCts_M[2]));

  /* Writes packet */
  addParamResult = syncWritePacket.addParam(ID_SHOULDER,  shoulderParam);
  addParamResult = syncWritePacket.addParam(ID_ELEVATION, elevateParam);
  addParamResult = syncWritePacket.addParam(ID_ELBOW,     elbowParam);
  dxlCommResult = syncWritePacket.txPacket();
  syncWritePacket.clearParam();

  return dxlCommResult;
}
