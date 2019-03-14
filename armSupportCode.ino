/* This code is combines the admittance control loop for the 3 DoF arm support. 
   This code takes the 1 DoF code and expands it to 3 DoF and includes the kinematics of the arm support.
   Functions needed are should be included in the folder.
   
   Script by erick nunez
   created: 1/24/2019

*/

// Libraries to add /////////////////////////////////////////////////////////////////////////////////
#include <math.h> 
#include <DynamixelSDK.h>

// OptoForce variables //////////////////////////////////////////////////////////////////////////////
#define xSensitivity 16.45
#define ySensitivity 18.42
#define zSensitivity 1.59
int16_t xRaw, yRaw, zRaw;
double  Fx, Fy, Fz, FxRaw, FyRaw, FzRaw;
double  xCal = 0, yCal = 0, zCal = 0;

// Dynamixel Variables /////////////////////////////////////////////////////////////////////////////
/* Communication Parameters */
#define PROTOCOL_VERSION 2.0
#define DXL_BUS_SERIAL   3
#define BAUDRATE         1000000
#define DEVICEPORT       "3"
/* Motor Parameters */
#define ID_SHOULDER      3
#define ID_ELBOW         7
/* Control Table Addresses */
#define ADDRESS_OPERATING_MODE   11
#define ADDRESS_MAX_POSITION     48
#define ADDRESS_MIN_POSITION     52
#define ADDRESS_TORQUE_ENABLE    64
#define ADDRESS_LED              65
#define ADDRESS_GOAL_VELOCITY    104
#define ADDRESS_PROFILE_VELOCITY 112
#define ADDRESS_GOAL_POSITION    116
#define ADDRESS_MOVING           122
#define ADDRESS_PRESENT_VELOCITY 128
#define ADDRESS_PRESENT_POSITION 132
/* Packet Parameters */
#define VELOCITY_CONTROL 1
#define POSITION_CONTROL 3
#define LENGTH_GOAL_POSITION      4
#define LENGTH_PROFILE_VELOCITY   4
#define LENGTH_PRESENT_POSITION   4
#define LENGTH_PRESENT_VELOCITY   4
#define ENABLE                    1
#define DISABLE                   0
#define ESC_ASCII_VALUE           0x1b
/* Parameters for calculations */
#define DEGREES_PER_COUNT 0.088
#define RPM_PER_COUNT     0.229
/* Motor Limits */
#define ELBOW_MIN_POS 1705
#define ELBOW_MAX_POS 3715
#define SHOULDER_MIN_POS 0
#define SHOULDER_MAX_POS 4095
#define ELBOW_MIN_VEL 0
#define ELBOW_MAX_VEL 300
#define SHOULDER_MIN_VEL 0
#define SHOULDER_MAX_VEL 300
/* Initialization of Variables */
int32_t presPosShoulder, presVelShoulder, presPosElbow, presVelElbow, presPosAct, presVelAct;
int     goalPosShoulder, goalVelShoulder, goalPosElbow, goalVelElbow, goalPosAct, goalVelAct;
byte    goalReturn;

// Admitance Control Variables //////////////////////////////////////////////////////////////////////
double xGoalPosSI, xGoalVelSI, yGoalPosSI, yGoalVelSI, zGoalPosSI, zGoalVelSI;
double xPresPosSI, xPresVelSI, yPresPosSI, yPresVelSI, zPresPosSI, zPresVelSI;
#define TIME         0.01 // loop time
#define MASS         5
#define DAMPING      0.5
#define GRAVITY      9.81

// Kinematic Variables and Constants ////////////////////////////////////////////////////////////////
#define SHOULDER_ELBOW_LINK 0.595
#define ELBOW_SENSOR_LINK   0.54
float presElbowAng, presElbowAngVel, presShoulderAng, presShoulderAngVel; //Radians, Rad/s
float goalElbowAng, goalElbowAngVel, goalShoulderAng, goalShoulderAngVel; //Radians, Rad/s

// Other Variables needed ///////////////////////////////////////////////////////////////////////////
unsigned long preTime, postTime;
int i, j, k;

// Setup function ///////////////////////////////////////////////////////////////////////////////////
void setup() {
  /* Serial Monitor */
  SerialUSB.begin();
  delay(10000);
  /* Dynamixel Serial Connection */
  SerialUSB.println(".....creating motor port.....");
  dxl.begin(3); //Baudrate: 1Mbps
  delay(100);
  SerialUSB.println(".....enabling motor.....");
  //while(!dxlEnable(POSITION_CONTROL, 1)){};
  /* Optoforce Serial Connection */
  SerialUSB.println(".....creating sensor port.....");
  Serial1.begin(1000000);
  delay(100);
  SerialUSB.println(".....configuring sensor.....");
  optoForceConfig();
  delay(100);
  SerialUSB.println(".....calibrating sensor.....");
  calibrateForceSensor();
  delay(1000);
  //got to initial position
  //SerialUSB.println(".....going to initial position.....");
  //goalVelElbow = MAX_VEL;        goalPosElbow = (ELBOW_MAX_POS + ELBOW_MIN_POS)/2;
  //goalVelShoulder = MAX_VEL;     goalPosShoulder = (SHOULDER_MAX_POS + SHOULDER_MIN_POS)/2;
  // goalReturn = dxlGoalVelPos(goalVelElbow, goalPosElbow, goalVelShoulder, goalPosShoulder);  
  
  SerialUSB.println("leaving setup");
}

// Main loop function ///////////////////////////////////////////////////////////////////////////////////
void loop() {
  /* Starts reading the sensor */
  preTime = millis();
 
  singleOptoForceRead();
  dxlPresVelPos();
  sensorOrientation();
  forwardKine();
  admittanceControl();
  inverseKine();
  
  if ((goalPosElbow <= ELBOW_MAX_POS & goalPosElbow >= ELBOW_MIN_POS) & (goalPosShoulder <= SHOULDER_MAX_POS & goalPosShoulder >= SHOULDER_MIN_POS)){
    if ((goalVelElbow <= ELBOW_MAX_VEL & goalVelElbow >= ELBOW_MIN_VEL) & (goalVelShoulder <= SHOULDER_MAX_VEL & goalVelShoulder >= SHOULDER_MIN_VEL)){
      goalReturn = dxlGoalVelPos(goalVelElbow, goalPosElbow, goalVelShoulder, goalPosShoulder);
    }
  }
  
  postTime = millis();
  //delay(10-(postTime-preTime));
  
  /*
  SerialUSB.print(Fz); SerialUSB.print("\t"); SerialUSB.print(zCal); SerialUSB.print("\t");
  SerialUSB.print(presPosSI); SerialUSB.print("\t");SerialUSB.print(presVelSI); SerialUSB.print("\t");
  SerialUSB.print(presPos); SerialUSB.print("\t");SerialUSB.print(presVel); SerialUSB.print("\t");
  SerialUSB.print(goalPosSI); SerialUSB.print("\t");SerialUSB.print(goalVelSI); SerialUSB.print("\t");
  SerialUSB.print(goalPos); SerialUSB.print("\t");SerialUSB.print(goalVel); SerialUSB.print("\t");
  */
  
  
  //SerialUSB.print(FxRaw); SerialUSB.print("\t");
  //SerialUSB.print(FyRaw); SerialUSB.print("\t");
  
  //SerialUSB.print(Fx); SerialUSB.print("\t");
  //SerialUSB.print(Fy); SerialUSB.print("\t");
  
  //SerialUSB.print(presElbowAng); SerialUSB.print("\t"); 
  //SerialUSB.print(presShoulderAng); SerialUSB.print("\t");
  
  //SerialUSB.print(presVelElbow); SerialUSB.print("\t"); 
  //SerialUSB.print(presVelShoulder); SerialUSB.print("\t");
  
  //SerialUSB.print(presPosElbow); SerialUSB.print("\t"); 
  //SerialUSB.print(presPosShoulder); SerialUSB.print("\t");
  
  SerialUSB.print(xPresPosSI); SerialUSB.print("\t");
  //SerialUSB.print(xPresVelSI); SerialUSB.print("\t");
  
  SerialUSB.print(yPresPosSI); SerialUSB.print("\t");
  //SerialUSB.print(yPresVelSI); SerialUSB.print("\t"); 
  
  SerialUSB.print(xGoalPosSI); SerialUSB.print("\t");
  //SerialUSB.print(xGoalVelSI); SerialUSB.print("\t");
  
  SerialUSB.print(yGoalPosSI); SerialUSB.print("\t");
  //SerialUSB.print(yGoalVelSI); SerialUSB.print("\t");
  
  SerialUSB.print(goalElbowAng); SerialUSB.print("\t"); 
  SerialUSB.print(goalShoulderAng); SerialUSB.print("\t");
  
  //SerialUSB.print(goalElbowAngVel); SerialUSB.print("\t");
  //SerialUSB.print(goalShoulderAngVel); SerialUSB.print("\t");
  
  //SerialUSB.print(goalVelElbow); SerialUSB.print("\t"); 
  //SerialUSB.print(goalVelShoulder); SerialUSB.print("\t");
  
  //SerialUSB.print(goalPosElbow); SerialUSB.print("\t");  
  //SerialUSB.print(goalPosShoulder); SerialUSB.print("\t");
  
  SerialUSB.print(postTime-preTime); SerialUSB.print("\t"); 
  SerialUSB.print(goalReturn); SerialUSB.print("\t");  
  SerialUSB.print("\n");
  
  
  goalReturn = 0;
  
}
