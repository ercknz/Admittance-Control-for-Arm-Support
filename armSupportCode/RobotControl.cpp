/* Class controls the arm support robot
   It includes the fKine and iKine as well as the
   functions needed to write and read to the motors
   Refer to github-Dynamixel for more information on dynamixel library.

   Created 10/28/2020
   Script by Erick Nunez
*/

#include "RobotControl.h"

RobotControl::RobotControl(const float A1, const float L1, const float A2, const float L2, const float Offset) {
  /* Robot Dimiensions */
  _A1A2     = A1 + A2;
  _L1       = L1;
  _L2       = L2;
  _Offset   = Offset;
  _PHI      = atan(Offset / L2);
  _H_OF_L2  = sqrt(pow(Offset, 2) + pow(L2, 2));
  
  /* JointSpace Limits */
  _Q1_MIN    = SHOULDER_MIN_POS * DEGREES_PER_COUNT * (PI / 180);
  _Q1_MAX    = SHOULDER_MAX_POS * DEGREES_PER_COUNT * (PI / 180);
  _Q4_MIN    = 0.0f;
  _Q4_MAX    = (ELBOW_MAX_POS - ELBOW_MIN_POS) * DEGREES_PER_COUNT * (PI / 180);
  _Q2_LIMIT  = (ELEVATION_MAX_POS - ELEVATION_CENTER) * DEGREES_PER_COUNT * (PI / 180.0) * (1 / ELEVATION_RATIO);
  _INNER_DIA = A1_LINK + L1_LINK + A2_LINK - L2_LINK;
}

void  RobotControl::iKine() {

}

void  RobotControl::fKine() {
  // Compute the XY positions from angles
  _x = _A1A2 * cos(q1) + _L1 * cos(q1) * cos(q2) + _Offset * sin(q1 + q4) + _L2 * cos(q1 + q4);
  _y = _A1A2 * sin(q1) + _L1 * sin(q1) * cos(q2) - _Offset * cos(q1 + q4) + _L2 * sin(q1 + q4);
  _z = _L1   * sin(q2);

  // Multiply velocities with Jacobian Matrix to find the XY velocities
  float J11 = - _A1A2   * sin(q1) - _L1 * sin(q1) * cos(q2) + _Offset * cos(q1 + q4) - _L2 * sin(q1 + q4);
  float J12 = - _L1     * cos(q1) * sin(q2);
  float J13 =   _Offset * cos(q1 + q4) - _L2 * sin(q1 + q4);
  float J21 =   _A1A2   * cos(q1) + _L1 * cos(q1) * cos(q2) + _Offset * sin(q1 + q4) + _L2 * cos(q1 + q4);
  float J22 = - _L1     * sin(q1) * sin(q2);
  float J23 =   _Offset * sin(q1 + q4) + _L2 * cos(q1 + q4);
  float J32 =   _L1     * cos(q2);  // J31 = J33 = 0 
  
  _xDot = q1Dot * J11 + q2Dot * J12 + q4Dot * J13;
  _yDot = q1Dot * J21 + q2Dot * J22 + q4Dot * J23;
  _zDot = q2Dot * J32;
}

void  RobotControl::EnableTorque() {

}

void  RobotControl::MotorConfig() {

}

void  RobotControl::ReadMotors(dynamixel::GroupSyncRead  &syncReadPacket) {
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

void  RobotControl::WriteMotors() {

}
