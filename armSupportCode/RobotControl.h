/* Class controls the arm support robot

   Created 10/28/2020
   Script by Erick Nunez
*/

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

class RobotControl {
  public:
          RobotControl(const float A1, const float L1, const float A2, const float L2, const float Offset);
    void  EnableTorque(bool state);
    void  MotorConfig();
    void  ReadRobot(bool &addParamResult, dynamixel::GroupSyncRead &syncReadPacket);
    void  WriteToRobot(float xyz[3], float xyzDot[3], bool &addParamResult, dynamixel::GroupSyncRead &syncReadPacket);
    float GetPresPos();
    float GetPresVel();
    float GetPresQ();
    float GetPresQDot();
    float GetGoalQCts();
    float GetGoalQDotCts();
    float GetGoalQ();
    float GetGoalQDot();
    
  private:
    void  fKine();
    void  iKine(float xyz[3], float xyzDot[3]);
    void  ReadMotors(bool &addParamResult, dynamixel::GroupSyncRead &syncReadPacket);
    int   WriteToMotors(bool &addParamResult, dynamixel::GroupSyncRead &syncReadPacket);

    const float _A1A2,      _L1,        _L2;
    const float _OFFSET,    _PHI,       _H_OF_L2;
    const float _Q1_MIN,    _Q1_MAX;
    const float _Q2_LIMIT;
    const float _Q4_MIN,    _Q4_MAX;
    const float _INNER_DIA; _Z_LIMIT;
    float J_M[3][3] = {0.0f};
    float qPresCts_M[3],  qDotPresCts_M[3];
    float qPres_M[3],     qDotPres_M[3];
    float xyzPres_M[3],   xyzDotPres_M[3];
    float qCts_M[3],      qDotCts_M[3];
    float q_M[3],         qDot_M[3];
    float xyz_M[3],       xyzDot_M[3];
};

#endif // ROBOT_CONTROL_H
