/* This tab is used to define the data structures that will be used throughout the 
   armSupportCode with will help organize the data. 

   Files are pushed to github:
   https://github.com/ercknz/Lab-ArmSupport

   Script by erick nunez
   created: 11/25/2019

*/

typedef struct{
  int32_t elbowPos;
  int32_t elbowVel;
  int32_t shldrPos;
  int32_t shldrVel;
} motorCountStruct;

typedef struct{
  float X;
  float Y;
  float Z;
} forceStruct;

typedef struct{
  float X;
  float Y;
  float U;
  float V;
} modelStruct;

typedef struct{
  float elbowAng;
  float elbowAVel;
  float shldrAng;
  float shldrAVel;
} angularStruct;

