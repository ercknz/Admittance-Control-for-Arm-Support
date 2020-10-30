/* Class controls the arm support robot
   It includes the fKine and iKine as well as the
   functions needed to write and read to the motors
   Refer to github-Dynamixel for more information on dynamixel library.

   q[0] = q1(shoulder)
   q[1] = q2(elevation)
   q[2] = q4(elbow)

   Created 10/28/2020
   Script by Erick Nunez
*/

#include "RobotControl.h"
#include "armSupportNamespace.h"

RobotControl::RobotControl(const float A1, const float L1, const float A2, const float L2, const float Offset) {
  /* Robot Dimiensions */
  _A1A2     = A1 + A2;
  _L1       = L1;
  _L2       = L2;
  _OFFSET   = Offset;
  _PHI      = atan(Offset / L2);
  _H_OF_L2  = sqrt(pow(Offset, 2) + pow(L2, 2));
  
  /* JointSpace Limits */
  _Q1_MIN    = SHOULDER_MIN_POS * DEGREES_PER_COUNT * (PI / 180);
  _Q1_MAX    = SHOULDER_MAX_POS * DEGREES_PER_COUNT * (PI / 180);
  _Q4_MIN    = 0.0f;
  _Q4_MAX    = (ELBOW_MAX_POS - ELBOW_MIN_POS) * DEGREES_PER_COUNT * (PI / 180);
  _Q2_LIMIT  = (ELEVATION_MAX_POS - ELEVATION_CENTER) * DEGREES_PER_COUNT * (PI / 180.0) * (1 / ELEVATION_RATIO);
  _INNER_DIA = A1 + L1 + A2 - L2;
}

void  RobotControl::iKine() {
   /* Checks if X and Y are both 0 */
  if ((abs(M.x) < 0.001) && (abs(M.y) < 0.001)) {
    Q.q1 = pres.q1;
    Q.q4 = Q4_MAX;
    M = forwardKine(Q);
  }

  /* Check Z limits */
  static float Z_LIMIT = L1_LINK * sin(Q2_LIMIT);
  if (M.z >  Z_LIMIT) M.z =  Z_LIMIT;
  if (M.z < -Z_LIMIT) M.z = -Z_LIMIT;

  /* Find variables based on Z */
  Q.q2 = asin(M.z / L1_LINK);
  float L1_XY = sqrt(pow(L1_LINK, 2) - pow(M.z, 2));

  /* Checks walls */
  float OUTER_DIA = A1_LINK + L1_XY + A2_LINK + H_OF_L2;
  float R = sqrt(pow(M.x, 2) + pow(M.y, 2));
  float alpha = atan2(M.y, M.x);
  if (alpha < 0) alpha += 2 * PI;
  if (R < INNER_DIA) {
    M.x = INNER_DIA * cos(alpha);
    M.y = INNER_DIA * sin(alpha);
    R = INNER_DIA;
  }
  if (R > OUTER_DIA) {
    M.x = OUTER_DIA * cos(alpha);
    M.y = OUTER_DIA * sin(alpha);
    R = OUTER_DIA;
  }

  /* Finds and checks Elbow Angle */
  float gamma = acos((pow((A1_LINK + L1_XY + A2_LINK), 2) + pow(H_OF_L2, 2) - pow(M.x, 2) - pow(M.y, 2)) / (2 * H_OF_L2 * (A1_LINK + L1_XY + A2_LINK)));

  Q.q4 = PI - gamma;
  if (Q.q4 < Q4_MIN) Q.q4 = Q4_MIN;
  if (Q.q4 > Q4_MAX) Q.q4 = Q4_MAX;

  /* Finds and checks shoulder angle */
  float beta = asin((H_OF_L2 * sin(gamma)) / R);
  Q.q1 = alpha - beta;
  if (Q.q1 < Q1_MIN) Q.q1 = Q1_MIN;
  if (Q.q1 > Q1_MAX) Q.q1 = Q1_MAX;

  /* Check for nans */
  if (Q.q1 != Q.q1) Q.q1 = pres.q1;
  if (Q.q4 != Q.q4) Q.q4 = pres.q4;

  /* Checks XYZ */
  modelSpace checkM = forwardKine(Q);
  if ((abs(M.x - checkM.x) > 0.001) || (abs(M.y - checkM.y) > 0.001) || (abs(M.y - checkM.y) > 0.001)) {
    M = checkM;
  }

  /* Solve for joint angular velocities (psuedo inverse Jacobian) */
  float detJ = (-L1_LINK * sin(Q.q1) - L2_LINK * sin(Q.q1 + Q.q4)) * (L2_LINK * cos(Q.q1 + Q.q4)) - (-L2_LINK * sin(Q.q1 + Q.q4)) * (L1_LINK * cos(Q.q1) + L2_LINK * cos(Q.q1 + Q.q4));
  Q.q1Dot = (M.xDot * (L2_LINK * cos(Q.q1 + Q.q4)) + M.yDot * (L2_LINK * sin(Q.q1 + Q.q4))) / detJ;
  Q.q2Dot = M.zDot / (L1_LINK * sqrt(1 - pow((M.z / L1_LINK), 2)));
  Q.q4Dot = -(M.xDot * (-L1_LINK * cos(Q.q1) - L2_LINK * cos(Q.q1 + Q.q4)) + M.yDot * (-L1_LINK * sin(Q.q1) - L2_LINK * sin(Q.q1 + Q.q4))) / detJ;
}

void  RobotControl::fKine() {
  // Compute the XY positions from angles
  xyz_M[0] = _A1A2 * cos(q1) + _L1 * cos(q1) * cos(q2) + _OFFSET * sin(q1 + q4) + _L2 * cos(q1 + q4);
  xyz_M[1] = _A1A2 * sin(q1) + _L1 * sin(q1) * cos(q2) - _OFFSET * cos(q1 + q4) + _L2 * sin(q1 + q4);
  xyz_M[2] = _L1   * sin(q2);

  // Multiply velocities with Jacobian Matrix to find the XY velocities
  J_M[0][0] = - _A1A2   * sin(q1) - _L1 * sin(q1) * cos(q2) + _OFFSET * cos(q1 + q4) - _L2 * sin(q1 + q4);
  J_M[0][1] = - _L1     * cos(q1) * sin(q2);
  J_M[0][2] =   _OFFSET * cos(q1 + q4) - _L2 * sin(q1 + q4);
  J_M[1][0] =   _A1A2   * cos(q1) + _L1 * cos(q1) * cos(q2) + _OFFSET * sin(q1 + q4) + _L2 * cos(q1 + q4);
  J_M[1][1] = - _L1     * sin(q1) * sin(q2);
  J_M[1][2] =   _OFFSET * sin(q1 + q4) + _L2 * cos(q1 + q4);
  J_M[2][1] =   _L1     * cos(q2);  // J31 = J33 = 0.0
  
  xyzDot_M[0] = q1Dot * J_M[0][0] + q2Dot * J_M[0][1] + q4Dot * J_M[0][2];
  xyzDot_M[1] = q1Dot * J_M[1][0] + q2Dot * J_M[1][1] + q4Dot * J_M[1][2];
  xyzDot_M[2] = q1Dot * J_M[2][0] + q2Dot * J_M[2][1] + q4Dot * J_M[2][2];
}

void  RobotControl::EnableTorque(uint8_t state) {
  using namespace ArmSupport
  int dxlCommResult;
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_TORQUE_ENABLE, state, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_TORQUE_ENABLE, state, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_TORQUE_ENABLE, state, &dxl_error);
}

void  RobotControl::MotorConfig() {
  using namespace ArmSupport
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

void  RobotControl::ReadMotors(dynamixel::GroupSyncRead  &syncReadPacket) {
  using namespace ArmSupport
  /* Read Position and Velocity */
  int dxlCommResult = syncReadPacket.txRxPacket();
  q4DotCts = syncReadPacket.getData(ID_ELBOW,        ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
  q4Cts    = syncReadPacket.getData(ID_ELBOW,        ADDRESS_PRESENT_POSITION, LEN_PRESENT_POSITION);
  q2DotCts = syncReadPacket.getData(ID_ELEVATION,    ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
  q2Cts    = syncReadPacket.getData(ID_ELEVATION,    ADDRESS_PRESENT_POSITION, LEN_PRESENT_POSITION);
  q1DotCts = syncReadPacket.getData(ID_SHOULDER,     ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
  q1Cts    = syncReadPacket.getData(ID_SHOULDER,     ADDRESS_PRESENT_POSITION, LEN_PRESENT_POSITION);

  /* Convert Motor Counts */
  q1      = (q1Cts) * DEGREES_PER_COUNT * (PI / 180.0);
  q2      = -(q2Cts - ELEVATION_CENTER) * DEGREES_PER_COUNT * (PI / 180.0) * (1 / ELEVATION_RATIO);
  q4      =  (q4Cts - ELBOW_MIN_POS) * DEGREES_PER_COUNT * (PI / 180.0);
  q1Dot   = q1DotCts * RPM_PER_COUNT * (2.0 * PI / 60.0);
  q2Dot   = q2DotCts * RPM_PER_COUNT * (2.0 * PI / 60.0) * (1 / ELEVATION_RATIO);
  q4Dot   = q4DotCts * RPM_PER_COUNT * (2.0 * PI / 60.0);
}

int  RobotControl::WriteMotors() {
  using namespace ArmSupport
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
  addParamResult = syncWritePacket.addParam(ID_SHOULDER,  shoulderParam);
  addParamResult = syncWritePacket.addParam(ID_ELEVATION, elevateParam);
  addParamResult = syncWritePacket.addParam(ID_ELBOW,     elbowParam);
  dxlCommResult = syncWritePacket.txPacket();
  syncWritePacket.clearParam();

  return dxlCommResult;
}
