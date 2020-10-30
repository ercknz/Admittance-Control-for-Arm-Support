/* Class controls the arm support robot

   Created 10/28/2020
   Script by Erick Nunez
*/

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

class RobotControl {
  public:
          RobotControl(const float A1, const float L1, const float A2, const float L2, const float Offset);
    void  iKine(float XYZ[3]);
    void  fKine();
    void  EnableTorque(bool state);
    void  MotorConfig();
    
  private:
    void  ReadMotors();
    void  WriteMotors();
    
    const float _A1A2,      _L1,        _L2;
    const float _OFFSET,    _PHI,       _H_OF_L2;
    const float _Q1_MIN,    _Q1_MAX;
    const float _Q2_LIMIT;
    const float _Q4_MIN,    _Q4_MAX;
    const float _INNER_DIA;
    float J_M[3][3] = {0.0f};
    float qCts_M[3];
    float qDotCts_M[3];
    float q_M[3];
    float qDot_M[3];
    float xyz_M[3];
    float xyzDot_M[3];
};

#endif // ROBOT_CONTROL_H
