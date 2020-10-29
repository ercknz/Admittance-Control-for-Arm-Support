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
    
    const float _A1;
    const float _L1;
    const float _A2;
    const float _L2;
    const float _Offset;
    const float _PHI;
    const float _H_OF_L2;
    float qCts[3];
    float qDotCts[3];
    float q[3];
    float qDot[3];
    float xyz[3];
    float xyzDot[3];
};

#endif // ROBOT_CONTROL_H
