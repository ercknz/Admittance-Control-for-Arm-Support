/* This code is combines the admittance control loop for the 3 DoF arm support. 
   This code takes the 1 DoF code and expands it to 3 DoF and includes the kinematics of the arm support.
   Functions needed are should be included in the folder.
   
   Script by erick nunez
   created: 1/24/2019

*/

// Libraries to add /////////////////////////////////////////////////////////////////////////////////
#include <DynamixelSDK.h>

// OptoForce variables ////////////////////////////////////////////////////////////////////////////// 
#define xSensitivity 16.45
#define ySensitivity 18.42
#define zSensitivity 1.59
int16_t xRaw, yRaw, zRaw;
float  Fx, Fy, Fz, FxRaw, FyRaw, FzRaw;
float  xCal = 0, yCal = 0, zCal = 0;

// Dynamixel Variables /////////////////////////////////////////////////////////////////////////////
/* Communication Parameters */
#define PROTOCOL_VERSION 2.0
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
#define VELOCITY_CONTROL      1
#define POSITION_CONTROL      3
#define LEN_GOAL_POSITION     4
#define LEN_PROFILE_VELOCITY  4
#define LEN_PRESENT_POSITION  4
#define LEN_PRESENT_VELOCITY  4
#define ENABLE                1
#define DISABLE               0
#define ESC_ASCII_VALUE       0x1b
/* Parameters for calculations */
#define DEGREES_PER_COUNT 0.088
#define RPM_PER_COUNT     0.229
/* Motor Limits */
#define ELBOW_MIN_POS     1705
#define ELBOW_MAX_POS     3715
#define SHOULDER_MIN_POS  0
#define SHOULDER_MAX_POS  4095
#define ELBOW_MIN_VEL     0
#define ELBOW_MAX_VEL     300
#define SHOULDER_MIN_VEL  0
#define SHOULDER_MAX_VEL  300
/* Initialization of Variables */
uint8_t elbowParam[8], shoulderParam[8];
int32_t presPosShoulder, presVelShoulder, presPosElbow, presVelElbow, presPosAct, presVelAct;
int32_t goalPosShoulder, goalVelShoulder, goalPosElbow, goalVelElbow, goalPosAct, goalVelAct;
bool    addParamResult;
uint8_t dxl_error = 0;
int     goalReturn; 
int     dxlCommResult = COMM_TX_FAIL;

// Admitance Control Variables //////////////////////////////////////////////////////////////////////
float xGoalPosSI, xGoalVelSI, yGoalPosSI, yGoalVelSI, zGoalPosSI, zGoalVelSI;
float xPresPosSI, xPresVelSI, yPresPosSI, yPresVelSI, zPresPosSI, zPresVelSI;
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

dynamixel::PortHandler *portHandler; 
dynamixel::PacketHandler *packetHandler; 

// Setup function ///////////////////////////////////////////////////////////////////////////////////
void setup() {
  /* Serial Monitor */
  Serial.begin(115200);
  while(!Serial);
  // Adds parameters to read packet 
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICEPORT);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  delay(100);
  /* Opens motor port */
  delay(100);
  if (portHandler -> openPort()){
    Serial.println("Opened the Dynamixel Port");
  }
  if (portHandler->setBaudRate(BAUDRATE)){
    Serial.println("Set baudrate!\n");
  }
  Serial.println(".....enabling motors.....");
  goalReturn = dxlAbling(POSITION_CONTROL, !ENABLE);
  Serial.println(goalReturn);
  /* Optoforce Serial Connection */
  Serial.println(".....creating sensor port.....");
  Serial1.begin(BAUDRATE);
  delay(100);
  Serial.println(".....configuring sensor.....");
  optoForceConfig();
  delay(100);
  Serial.println(".....calibrating sensor.....");
  calibrateForceSensor(xCal, yCal, zCal);  
  delay(1000);
  Serial.println("leaving setup");
}

// Main loop function ///////////////////////////////////////////////////////////////////////////////////
void loop() {
  dynamixel::GroupSyncWrite syncWritePacket(portHandler, packetHandler, ADDRESS_PROFILE_VELOCITY, LEN_PROFILE_VELOCITY + LEN_GOAL_POSITION);
  dynamixel::GroupSyncRead  syncReadPacket(portHandler, packetHandler, ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION);
  addParamResult = syncReadPacket.addParam(ID_SHOULDER);
  addParamResult = syncReadPacket.addParam(ID_ELBOW);
  while(1){
    /* Starts reading the sensor */
    preTime = millis();
 
    singleOptoForceRead(xRaw, yRaw, zRaw, FxRaw, FyRaw, FzRaw);
    readPresentPacket(syncReadPacket, presVelElbow, presPosElbow, presVelShoulder, presPosShoulder);
    sensorOrientation(FxRaw, FyRaw, presPosElbow, presPosShoulder, Fx, Fy, presElbowAng, presShoulderAng);
    forwardKine();
    admittanceControl(Fx, xPresPosSI, xPresVelSI, xGoalPosSI, xGoalVelSI, Fy, yPresPosSI, yPresVelSI, yGoalPosSI, yGoalVelSI);
    inverseKine();
  
    if ((goalPosElbow <= ELBOW_MAX_POS & goalPosElbow >= ELBOW_MIN_POS) & (goalPosShoulder <= SHOULDER_MAX_POS & goalPosShoulder >= SHOULDER_MIN_POS)){
      if ((goalVelElbow <= ELBOW_MAX_VEL & goalVelElbow >= ELBOW_MIN_VEL) & (goalVelShoulder <= SHOULDER_MAX_VEL & goalVelShoulder >= SHOULDER_MIN_VEL)){
        goalReturn = writeGoalPacket(syncWritePacket, goalVelElbow, goalPosElbow, goalVelShoulder, goalPosShoulder);
      }
    }
  
    postTime = millis();
    //delay(10-(postTime-preTime));
    
    Serial.print(FxRaw); Serial.print("\t");
    Serial.print(FyRaw); Serial.print("\t");
  
    //Serial.print(Fx); Serial.print("\t");
    //Serial.print(Fy); Serial.print("\t");
  
    //Serial.print(presElbowAng); Serial.print("\t"); 
    //Serial.print(presShoulderAng); Serial.print("\t");
  
    Serial.print(presVelElbow); Serial.print("\t"); 
    Serial.print(presVelShoulder); Serial.print("\t");
  
    Serial.print(presPosElbow); Serial.print("\t"); 
    Serial.print(presPosShoulder); Serial.print("\t");
  
    //Serial.print(xPresPosSI); Serial.print("\t");
    //Serial.print(xPresVelSI); Serial.print("\t");
  
    //Serial.print(yPresPosSI); Serial.print("\t");
    //Serial.print(yPresVelSI); Serial.print("\t"); 
  
    //Serial.print(xGoalPosSI); Serial.print("\t");
    //Serial.print(xGoalVelSI); Serial.print("\t");
  
    //Serial.print(yGoalPosSI); Serial.print("\t");
    //Serial.print(yGoalVelSI); Serial.print("\t");
  
    //Serial.print(goalElbowAng); Serial.print("\t"); 
    //Serial.print(goalShoulderAng); Serial.print("\t");
  
    //Serial.print(goalElbowAngVel); Serial.print("\t");
    //Serial.print(goalShoulderAngVel); Serial.print("\t");
  
    Serial.print(goalVelElbow); Serial.print("\t"); 
    Serial.print(goalVelShoulder); Serial.print("\t");
  
    Serial.print(goalPosElbow); Serial.print("\t");  
    Serial.print(goalPosShoulder); Serial.print("\t");
  
    Serial.print(postTime-preTime); Serial.print("\t"); 
    Serial.print(goalReturn); Serial.print("\t");  
    Serial.print("\n");
  }
}
