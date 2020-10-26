/* This tab is used to define the data structures that will be used throughout the 
   armSupportCode with will help organize the data. 

   Files are pushed to github:
   https://github.com/ercknz/Admittance-Control-for-Arm-Support

   Script by erick nunez
   created: 11/25/2019

*/

/********** Force Sensor Data Structure **********/
#ifndef DATASTRUCTS
#define DATASTRUCTS

typedef struct{
  float X;
  float Y;
  float Z;
} forceStruct;

/********** Model Space Data Structure **********/
typedef struct{
  float x;
  float y;
  float z;
  float xDot;
  float yDot;
  float zDot;
} modelSpace;

/********** Joint Space Data Structure **********/
typedef struct{
  // Joint Motor Position and velocity counts
  int32_t q1Cts;
  int32_t q1DotCts;
  int32_t q2Cts;
  int32_t q2DotCts;
  int32_t q4Cts;
  int32_t q4DotCts;
  // Joint angles and angular velocities
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

#endif // DATASTRUCTS

