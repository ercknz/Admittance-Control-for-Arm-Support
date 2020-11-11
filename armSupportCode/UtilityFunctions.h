/* Utility functions used with the arm support robot.

   Created 10/28/2020
   by Erick Nunez
*/

#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

#include "ForceSensor.h"
#include "AdmittanceModel.h"
#include "RobotControl.h"

int16_t bytesToCounts(byte hByte, byte lByte);

void loggingFunc(unsigned long totalTime, ForceSensor &OptoSensor, AdmittanceModel &Model, RobotControl &Robot, unsigned long loopTime);

//typedef struct{
//  float X;
//  float Y;
//  float Z;
//} ForceStruct;
//
//typedef struct{
//  float x;
//  float y;
//  float z;
//  float xDot;
//  float yDot;
//  float zDot;
//} ModelSpace;
//
//typedef struct{
//  float q1;
//  float q2;
//  float q4;
//  float q1Dot;
//  float q2Dot;
//  float q4Dot;
//} JointSpace;

#endif // UTILITY_FUNCTIONS_H
