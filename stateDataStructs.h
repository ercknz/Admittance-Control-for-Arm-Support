/* This tab is used to define the data structures that will be used throughout the 
   armSupportCode with will help organize the data. 

   Files are pushed to github:
   https://github.com/ercknz/Lab-ArmSupport

   Script by erick nunez
   created: 11/25/2019

*/

typedef struct{
  float X;
  float Y;
  float Z;
} forceStruct;

typedef struct{
  float x;
  float y;
  float z;
  float xDot;
  float yDot;
  float zDot;
} modelSpace;

typedef struct{
  float q1;       // Shoulder Angle
  float q1Dot;    // Shoulder Angular Velocity
  float q2;       // Elevation Angle
  float q2Dot;    // Elevation Angular Velocity
  float q4;       // Elbow Angle
  float q4Dot;    // Elbow Angular Velocity
  /* 
   *  q3 is the other joint at the 4 bar linkage and is equal to -q2
   *  q3 is a redundant joint
   */
} jointSpace;

