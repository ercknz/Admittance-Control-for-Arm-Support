/* Class controls the arm support robot

   Created 10/28/2020
   Script by Erick Nunez
*/

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

class RobotControl {
  public:
          RobotControl();
    void  iKine(float XYZ[3]);
    void  fKine();
    void  EnableTorque(bool state);
    void  MotorConfig();
    void  ReadMotors();
    void  WriteMotors();
  private:
    const float _A1Link;
    const float _L1Link;
    const float _A2Link;
    const float _L2Link;
    const float _PHI;
    const float _H_OF_L2
    float q1Cts,    q2Cts,    q4Cts;
    float q1DotCts, q2DotCts, q4DotCts;
    float q1,       q2,       q4;
    float q1Dot,    q2Dot,    q4Dot;
};

#endif // ROBOT_CONTROL_H
