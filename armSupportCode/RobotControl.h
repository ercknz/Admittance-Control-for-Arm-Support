/* Class controls the arm support robot

   Created 10/28/2020
   Script by Erick Nunez
*/

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <DynamixelSDK.h>

class RobotControl {
  public:
          RobotControl(const float A1, const float L1, const float A2, const float L2, const float Offset);
    void  EnableTorque(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler  *packetHandler, uint8_t state);
    void  MotorConfig(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler  *packetHandler);
    void  ReadRobot(dynamixel::GroupSyncRead &syncReadPacket);
    void  WriteToRobot(float *xyz, float *xyzDot, bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket);
    float *   GetPresQ();
    float *   GetPresQDot();
    int32_t * GetPresQCts();
    int32_t * GetPresQDotCts();
    float *   GetPresPos();
    float *   GetPresVel();
    int32_t * GetGoalQCts();
    int32_t * GetGoalQDotCts();
    float *   GetGoalQ();
    float *   GetGoalQDot();
    float *   GetGoalPos();
    float *   GetGoalVel();
    void      CalculateSpringForce(float *forces); 
    float     GetSpringForce();
    void      SetScalingFactor(float newScalingFactor);
    
  protected:
    void  fKine();
    void  iKine(float *modelXYZ, float *modelXYZDot);
    void  ReadMotors(dynamixel::GroupSyncRead &syncReadPacket);
    int   WriteToMotors(bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket);

    const float  _A1A2,      _L1,        _L2;
    const double _OFFSET,    _PHI,       _H_OF_L2;
    const double _Q1_MIN,    _Q1_MAX;
    const double _Q2_LIMIT;
    const double _Q4_MIN,    _Q4_MAX;
    const double _INNER_R,   _Z_LIMIT;
    const double _SPRING_Li, _BETAi, _SPRING_Fi;
    float J_M[3][3] = {{0.0f}};
    int32_t qPresCts_M[3],  qDotPresCts_M[3];
    float   qPres_M[3],     qDotPres_M[3];
    float   xyzPres_M[3],   xyzDotPres_M[3];
    int32_t qCts_M[3],      qDotCts_M[3];
    float   q_M[3],         qDot_M[3];
    float   xyz_M[3],       xyzDot_M[3];
    float   springF_M;
    float   scalingFactor_M;
    uint8_t dxl_error = 0;
};

#endif // ROBOT_CONTROL_H
