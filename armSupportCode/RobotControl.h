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
    float* GetPresQ();
    float* GetPresQDot();
    int  * GetPresQCts();
    int  * GetPresQDotCts();
    float* GetPresPos();
    float* GetPresVel();
    int  * GetGoalQCts();
    int  * GetGoalQDotCts();
    float* GetGoalQ();
    float* GetGoalQDot();
    
  private:
    void  fKine();
    void  iKine(float *xyz, float *xyzDot);
    void  ReadMotors(dynamixel::GroupSyncRead &syncReadPacket);
    int   WriteToMotors(bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket);

    const float _A1A2,      _L1,        _L2;
    const float _OFFSET,    _PHI,       _H_OF_L2;
    const float _Q1_MIN,    _Q1_MAX;
    const float _Q2_LIMIT;
    const float _Q4_MIN,    _Q4_MAX;
    const float _INNER_R,   _Z_LIMIT;
    float J_M[3][3] = {{0.0f}};
    int   qPresCts_M[3],  qDotPresCts_M[3];
    float qPres_M[3],     qDotPres_M[3];
    float xyzPres_M[3],   xyzDotPres_M[3];
    int   qCts_M[3],      qDotCts_M[3];
    float q_M[3],         qDot_M[3];
    float xyz_M[3],       xyzDot_M[3];
    uint8_t dxl_error = 0;
};

#endif // ROBOT_CONTROL_H
