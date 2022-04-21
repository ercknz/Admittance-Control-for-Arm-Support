/* Class controls the arm support robot
   It includes the fKine and iKine as well as the
   functions needed to write and read to the motors
   Refer to github-Dynamixel for more information on dynamixel library.

   Class arrays are used as follows:
   xyz[3]     = {x, y, x}; 
   xyzDot[3]  = {xDot, yDot, zDot};
   q[3]       = {q1(shoulder), q2(elevation), q4(elbow)};
   qDot[3]    = {q1Dot(shoulder), q2Dot(elevation), q4Dot(elbow)}

   Created 10/28/2020
   Script by Erick Nunez
*/

#include <DynamixelSDK.h>
#include <Arduino.h>
#include "RobotControl.h"
#include "armSupportNamespace.h"

/* ---------------------------------------------------------------------------------------/
/ Arm Support Constructor ----------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
RobotControl::RobotControl(const float A1, const float L1, const float A2, const float L2, const float Offset)
  :_A1A2{A1 + A2},
  _L1{L1},
  _L2{L2},
  _OFFSET{Offset},
  _PHI{atan(Offset / L2)},
  _H_OF_L2{sqrt(pow(Offset, 2) + pow(L2, 2))},
  _Q1_MIN{ASR::SHOULDER_MIN_POS * ASR::DEGREES_PER_COUNT * (PI / 180.0)},
  _Q1_MAX{ASR::SHOULDER_MAX_POS * ASR::DEGREES_PER_COUNT * (PI / 180.0)},
  _Q2_LIMIT{abs((ASR::ELEVATION_MAX_POS - ASR::ELEVATION_CENTER) * ASR::DEGREES_PER_COUNT * (PI / 180.0) * (1/ASR::ELEVATION_RATIO))},
  _Q4_MIN{(ASR::ELBOW_MIN_POS - ASR::ELBOW_MIN_POS) * ASR::DEGREES_PER_COUNT * (PI / 180.0)},
  _Q4_MAX{(ASR::ELBOW_MAX_POS - ASR::ELBOW_MIN_POS) * ASR::DEGREES_PER_COUNT * (PI / 180.0)},
  _INNER_R{A1 + L1 + A2 - L2},
  _Z_LIMIT{abs(L1 * sin((ASR::ELEVATION_MAX_POS - ASR::ELEVATION_CENTER) * ASR::DEGREES_PER_COUNT * (PI / 180.0) * (1/ASR::ELEVATION_RATIO)))},
  _SPRING_Li{sqrt(pow(ASR::SPRING_SIDE_A,2) + pow(ASR::SPRING_SIDE_B,2) + 2 * ASR::SPRING_SIDE_A * ASR::SPRING_SIDE_B * ASR::COS_SIN_45)},
  _BETAi{asin((ASR::SPRING_SIDE_A/_SPRING_Li) * - ASR::COS_SIN_45)},
  _SPRING_Fi{ASR::SPRING_KS * (ASR::SPRING_XI - ASR::SPRING_X0) * sin(_BETAi + ASR::DEG_TO_RAD_45)}
{
  scalingFactor_M = ASR::SPRING_FORCE_SCALING_FACTOR;
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Get Member functions -------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
int32_t* RobotControl::GetPresQCts(){ 
  return qPresCts_M;
} 

int32_t* RobotControl::GetPresQDotCts(){
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

int32_t* RobotControl::GetGoalQCts(){ 
  return qCts_M;
}

int32_t* RobotControl::GetGoalQDotCts(){
  return qDotCts_M;
}

float* RobotControl::GetGoalQ(){
  return q_M;
}

float* RobotControl::GetGoalQDot(){
  return qDot_M;
}

float* RobotControl::GetGoalPos(){ 
  return xyz_M;
}

float* RobotControl::GetGoalVel(){
  return xyzDot_M;
}

float RobotControl::GetSpringForce(){
  return springF_M;
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Motors Reading/Writing -----------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void RobotControl::ReadRobot(dynamixel::GroupSyncRead &syncReadPacket){
  ReadMotors(syncReadPacket);
  fKine();
}

void RobotControl::WriteToRobot(float *xyz, float *xyzDot, bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket){
  iKine(xyz, xyzDot);
  int returnInt = WriteToMotors(addParamResult, syncWritePacket);
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Inverse Kinematics Member function -----------------------------------------/
/----------------------------------------------------------------------------------------*/
void RobotControl::iKine(float *modelXYZ, float *modelXYZDot) {
  /*
    TAKE NOTE!
    xyz_M[3] = {x, y, z}
    q_M[3] = {q1, q2, q4} = {shoulder, elevation, elbow}
  */
  float L1_XY, OUTER_R, R, alpha, presR, presAlpha, beta, gamma, detJ;
  for (int i=0; i<3; i++){
    xyz_M[i]    = xyzPres_M[i] + modelXYZ[i];
    xyzDot_M[i] = modelXYZDot[i];
  }

  /* Check Z limits */
  if (xyz_M[2] >  _Z_LIMIT) xyz_M[2] =  _Z_LIMIT;
  if (xyz_M[2] < -_Z_LIMIT) xyz_M[2] = -_Z_LIMIT;

  /* Find variables based on Z */
  q_M[1]  = asin(xyz_M[2] / _L1);
  L1_XY   = sqrt(pow(_L1, 2) - pow(xyz_M[2], 2));

   /* Checks if X and Y are both 0 */
  if ((abs(xyz_M[0]) < 0.001) && (abs(xyz_M[1]) < 0.001)) {
    q_M[0] = qPres_M[0];
    q_M[2] = _Q4_MAX;
    xyz_M[0] = _A1A2 * cos(q_M[0]) + _L1 * cos(q_M[0]) * cos(q_M[1]) + _OFFSET * sin(q_M[0] + q_M[2]) + _L2 * cos(q_M[0] + q_M[2]);
    xyz_M[1] = _A1A2 * sin(q_M[0]) + _L1 * sin(q_M[0]) * cos(q_M[1]) - _OFFSET * cos(q_M[0] + q_M[2]) + _L2 * sin(q_M[0] + q_M[2]);
  }

  /* R and Alpha */
  R       = sqrt(pow(xyz_M[0], 2) + pow(xyz_M[1], 2));
  alpha   = atan2(xyz_M[1], xyz_M[0]);
  if (alpha < 1.0f) alpha += 2 * PI;
  presR       = sqrt(pow(xyzPres_M[0], 2) + pow(xyzPres_M[1], 2));
  presAlpha   = atan2(xyzPres_M[1], xyzPres_M[0]);
  if (presAlpha < 1.0f) presAlpha += 2 * PI;

  /* Checks walls */
  OUTER_R = _A1A2 + _H_OF_L2 + L1_XY;
  if (R < _INNER_R) {
    R         = _INNER_R;
    xyz_M[0]  = _INNER_R * cos(alpha);
    xyz_M[1]  = _INNER_R * sin(alpha);
  }
  if (R > OUTER_R) {
    R         = OUTER_R;
    xyz_M[0]  = OUTER_R * cos(alpha);
    xyz_M[1]  = OUTER_R * sin(alpha);
  } 
  
  /* Finds and checks Elbow Angle */
  if ((abs(R-presR) < 0.01) && (alpha < presAlpha)){
    q_M[2] = qPres_M[2];
    gamma = PI - q_M[2];
  } else {
    gamma = acos((pow((_A1A2 + L1_XY), 2) + pow(_H_OF_L2, 2) - pow(xyz_M[0], 2) - pow(xyz_M[1], 2)) / (2 * _H_OF_L2 * (_A1A2 + L1_XY)));
    q_M[2] = PI - gamma;
  }

  /* Finds and checks shoulder angle */
  beta = asin((_H_OF_L2 * sin(gamma)) / R);
  q_M[0] = alpha - beta;

  /* Check for nans */
  if (q_M[0] != q_M[0]) q_M[0] = qPres_M[0];
  if (q_M[2] != q_M[2]) q_M[2] = qPres_M[2];

  /* Check Jointspace Limits */
  if (q_M[2] < _Q4_MIN) q_M[2] = _Q4_MIN;
  if (q_M[2] > _Q4_MAX) q_M[2] = _Q4_MAX;
  if (q_M[0] < _Q1_MIN) q_M[0] = _Q1_MIN;
  if (q_M[0] > _Q1_MAX) q_M[0] = _Q1_MAX;

  /* Solve for joint angular velocities (psuedo inverse Jacobian) */
  detJ      = (-_L1 * sin(q_M[0]) - _L2 * sin(q_M[0] + q_M[2])) * (_L2 * cos(q_M[0] + q_M[2])) - (-_L2 * sin(q_M[0] + q_M[2])) * (_L1 * cos(q_M[0]) + _L2 * cos(q_M[0] + q_M[2]));
  qDot_M[0] = (xyzDot_M[0] * (_L2 * cos(q_M[0] + q_M[2])) + xyzDot_M[1] * (_L2 * sin(q_M[0] + q_M[2]))) / detJ;
  qDot_M[1] = xyzDot_M[2] / (_L1 * sqrt(1 - pow((xyz_M[2] / _L1), 2)));
  qDot_M[2] = (xyzDot_M[0] * (-_L1 * cos(q_M[0]) - _L2 * cos(q_M[0] + q_M[2])) + xyzDot_M[1] * (-_L1 * sin(q_M[0]) - _L2 * sin(q_M[0] + q_M[2]))) / detJ;

  /* Convert to Motor Counts */
  qCts_M[0]    = q_M[0] * (180.0 / PI) / ASR::DEGREES_PER_COUNT;
  qCts_M[1]    = ASR::ELEVATION_CENTER - (q_M[1] * ASR::ELEVATION_RATIO * (180.0 / PI) / ASR::DEGREES_PER_COUNT);
  qCts_M[2]    = ASR::ELBOW_MIN_POS + q_M[2] * (180.0 / PI) / ASR::DEGREES_PER_COUNT;
  qDotCts_M[0] = abs(qDot_M[0] * (60.0 / (2.0 * PI)) / ASR::RPM_PER_COUNT);
  qDotCts_M[1] = abs(qDot_M[1] * (60.0 / (2.0 * PI)) / ASR::RPM_PER_COUNT) * ASR::ELEVATION_RATIO;
  qDotCts_M[2] = abs(qDot_M[2] * (60.0 / (2.0 * PI)) / ASR::RPM_PER_COUNT);
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Forward Kinematics Member Function -----------------------------------------/
/----------------------------------------------------------------------------------------*/
void  RobotControl::fKine() {
  /* Convert from Motor Counts */
  qPres_M[0]      =  (qPresCts_M[0]) * ASR::DEGREES_PER_COUNT * (PI / 180.0);
  qPres_M[1]      = -(qPresCts_M[1] - ASR::ELEVATION_CENTER) * ASR::DEGREES_PER_COUNT * (PI / 180.0) * (1/ASR::ELEVATION_RATIO);
  qPres_M[2]      =  (qPresCts_M[2] - ASR::ELBOW_MIN_POS) * ASR::DEGREES_PER_COUNT * (PI / 180.0);
  qDotPres_M[0]   = qDotPresCts_M[0] * ASR::RPM_PER_COUNT * (2.0 * PI / 60.0);
  qDotPres_M[1]   = qDotPresCts_M[1] * ASR::RPM_PER_COUNT * (2.0 * PI / 60.0) * (1/ASR::ELEVATION_RATIO);
  qDotPres_M[2]   = qDotPresCts_M[2] * ASR::RPM_PER_COUNT * (2.0 * PI / 60.0);
  
  /* Calculates the Taskspace Position */
  xyzPres_M[0] = _A1A2 * cos(qPres_M[0]) + _L1 * cos(qPres_M[0]) * cos(qPres_M[1]) + _OFFSET * sin(qPres_M[0] + qPres_M[2]) + _L2 * cos(qPres_M[0] + qPres_M[2]);
  xyzPres_M[1] = _A1A2 * sin(qPres_M[0]) + _L1 * sin(qPres_M[0]) * cos(qPres_M[1]) - _OFFSET * cos(qPres_M[0] + qPres_M[2]) + _L2 * sin(qPres_M[0] + qPres_M[2]);
  xyzPres_M[2] =   _L1 * sin(qPres_M[1]);

  /* Calculates Jacobian Matrix */
  J_M[0][0] = - _A1A2   * sin(qPres_M[0]) - _L1 * sin(qPres_M[0]) * cos(qPres_M[1]) + _OFFSET * cos(qPres_M[0] + qPres_M[2]) - _L2 * sin(qPres_M[0] + qPres_M[2]);
  J_M[0][1] = - _L1     * cos(qPres_M[0]) * sin(qPres_M[1]);
  J_M[0][2] =   _OFFSET * cos(qPres_M[0] + qPres_M[2]) - _L2 * sin(qPres_M[0] + qPres_M[2]);
  J_M[1][0] =   _A1A2   * cos(qPres_M[0]) + _L1 * cos(qPres_M[0]) * cos(qPres_M[1]) + _OFFSET * sin(qPres_M[0] + qPres_M[2]) + _L2 * cos(qPres_M[0] + qPres_M[2]);
  J_M[1][1] = - _L1     * sin(qPres_M[0]) * sin(qPres_M[1]);
  J_M[1][2] =   _OFFSET * sin(qPres_M[0] + qPres_M[2]) + _L2 * cos(qPres_M[0] + qPres_M[2]);
  J_M[2][1] =   _L1     * cos(qPres_M[1]);  // J31 = J33 = 0.0

  /* Calcaultes Taskspace Velociies */
  xyzDotPres_M[0] = qDotPres_M[0] * J_M[0][0] + qDotPres_M[1] * J_M[0][1] + qDotPres_M[2] * J_M[0][2];
  xyzDotPres_M[1] = qDotPres_M[0] * J_M[1][0] + qDotPres_M[1] * J_M[1][1] + qDotPres_M[2] * J_M[1][2];
  xyzDotPres_M[2] = qDotPres_M[0] * J_M[2][0] + qDotPres_M[1] * J_M[2][1] + qDotPres_M[2] * J_M[2][2];
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Spring Force Member Functions ----------------------------------------------/
/----------------------------------------------------------------------------------------*/
void RobotControl::CalculateSpringForce(float *forces){
  /* Calculated Variables */
  float maxCompensation = 0.5;
  float phaseCompensation;
  if (forces[2] < 0.0){
    phaseCompensation = maxCompensation * 0.40;
  } else {
    phaseCompensation = 0.0;
  }
  float alpha = 90.0 - qPres_M[1];
  float springLength = sqrt(pow(ASR::SPRING_SIDE_A,2) + pow(ASR::SPRING_SIDE_B,2) - 2 * ASR::SPRING_SIDE_A * ASR::SPRING_SIDE_B * cos(alpha));
  float beta = asin((ASR::SPRING_SIDE_A/springLength) * sin(alpha));
  springF_M = scalingFactor_M * phaseCompensation *(ASR::SPRING_KS * (springLength - _SPRING_Li + ASR::SPRING_XI - ASR::SPRING_X0) * sin(beta - qPres_M[1]) - _SPRING_Fi);
}

void RobotControl::SetScalingFactor(float newScalingFactor){
  scalingFactor_M = newScalingFactor;
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support DXL Torque Enabling Member Function ----------------------------------------/
/----------------------------------------------------------------------------------------*/
void  RobotControl::EnableTorque(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler  *packetHandler, uint8_t state) {
  /* Modes to Select from:
   *  [5]:  Full Admittance Control (Shoulder, Elbow, and Elevation ENABLED)
   *  [10]: Planar Admittance Control ONLY (Shoulder and  Elbow ENABLED, Elevation DISABLED)
   *  [15]: Vertical Admittance Control ONLY (Elevation ENABLED, Shoulder and Elbow DISABLED)
   *  [20]: Fully Passive (Shoulder, Elbow, and Elevation DISABLED)
   */
  using namespace ASR;
  int dxlCommResult;
  if ((state == 5)||(state == 10)){
    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_TORQUE_ENABLE, ENABLE, &dxl_error);
    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_TORQUE_ENABLE, ENABLE, &dxl_error);
  }
  if ((state == 5)||(state == 15)){
    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_TORQUE_ENABLE, ENABLE, &dxl_error);
  }
  if ((state == 20)||(state == 10)){
    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_TORQUE_ENABLE, DISABLE, &dxl_error);
  }
  if ((state == 20)||(state == 15)){
    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_TORQUE_ENABLE, DISABLE, &dxl_error);
    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_TORQUE_ENABLE, DISABLE, &dxl_error);
  }
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support DXL Configuration Member Function ------------------------------------------/
/----------------------------------------------------------------------------------------*/
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

/* ---------------------------------------------------------------------------------------/
/ Arm Support DXL Read Member Function ---------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void  RobotControl::ReadMotors(dynamixel::GroupSyncRead  &syncReadPacket) {
  /* Read Position and Velocity */
  int dxlCommResult = syncReadPacket.txRxPacket();
  qPresCts_M[0]    = syncReadPacket.getData(ASR::ID_SHOULDER,  ASR::ADDRESS_PRESENT_POSITION, ASR::LEN_PRESENT_POSITION);
  qPresCts_M[1]    = syncReadPacket.getData(ASR::ID_ELEVATION, ASR::ADDRESS_PRESENT_POSITION, ASR::LEN_PRESENT_POSITION);
  qPresCts_M[2]    = syncReadPacket.getData(ASR::ID_ELBOW,     ASR::ADDRESS_PRESENT_POSITION, ASR::LEN_PRESENT_POSITION);
  qDotPresCts_M[0] = syncReadPacket.getData(ASR::ID_SHOULDER,  ASR::ADDRESS_PRESENT_VELOCITY, ASR::LEN_PRESENT_VELOCITY);
  qDotPresCts_M[1] = syncReadPacket.getData(ASR::ID_ELEVATION, ASR::ADDRESS_PRESENT_VELOCITY, ASR::LEN_PRESENT_VELOCITY);
  qDotPresCts_M[2] = syncReadPacket.getData(ASR::ID_ELBOW,     ASR::ADDRESS_PRESENT_VELOCITY, ASR::LEN_PRESENT_VELOCITY);
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support DXL Write Member Function --------------------------------------------------/
/----------------------------------------------------------------------------------------*/
int  RobotControl::WriteToMotors(bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket) {
  int dxlCommResult;
  uint8_t elbowParam[4], shoulderParam[4], elevateParam[4];

  /* Shoulder Goal Position Packet */
  shoulderParam[0] = DXL_LOBYTE(DXL_LOWORD(qCts_M[0]));
  shoulderParam[1] = DXL_HIBYTE(DXL_LOWORD(qCts_M[0]));
  shoulderParam[2] = DXL_LOBYTE(DXL_HIWORD(qCts_M[0]));
  shoulderParam[3] = DXL_HIBYTE(DXL_HIWORD(qCts_M[0]));

  /* Elevation Goal Position Packet */
  elevateParam[0] = DXL_LOBYTE(DXL_LOWORD(qCts_M[1]));
  elevateParam[1] = DXL_HIBYTE(DXL_LOWORD(qCts_M[1]));
  elevateParam[2] = DXL_LOBYTE(DXL_HIWORD(qCts_M[1]));
  elevateParam[3] = DXL_HIBYTE(DXL_HIWORD(qCts_M[1]));

  /* Elbow Goal Position Packet */
  elbowParam[0] = DXL_LOBYTE(DXL_LOWORD(qCts_M[2]));
  elbowParam[1] = DXL_HIBYTE(DXL_LOWORD(qCts_M[2]));
  elbowParam[2] = DXL_LOBYTE(DXL_HIWORD(qCts_M[2]));
  elbowParam[3] = DXL_HIBYTE(DXL_HIWORD(qCts_M[2]));

  /* Writes Packets */
  addParamResult = syncWritePacket.addParam(ASR::ID_SHOULDER,  shoulderParam);
  addParamResult = syncWritePacket.addParam(ASR::ID_ELEVATION, elevateParam);
  addParamResult = syncWritePacket.addParam(ASR::ID_ELBOW,     elbowParam);
  dxlCommResult = syncWritePacket.txPacket();
  syncWritePacket.clearParam();

  return dxlCommResult;
}
