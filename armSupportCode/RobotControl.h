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
    const float _A1;
    const float _L1;
    const float _A2;
    const float _L2;
    const float _Offset;
    const float _PHI;
    const float _H_OF_L2;
    float q1Cts,    q2Cts,    q4Cts;
    float q1DotCts, q2DotCts, q4DotCts;
    float q1,       q2,       q4;
    float q1Dot,    q2Dot,    q4Dot;
    float _x,       _y,       _z;
    float _xDot,    _yDot,    _zDot;

    void  ReadMotors();
    void  WriteMotors();
};

#endif // ROBOT_CONTROL_H
