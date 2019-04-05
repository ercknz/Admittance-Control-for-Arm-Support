/* This code is combines the admittance control loop for the 3 DoF arm support. 
   This code takes the 1 DoF code and expands it to 3 DoF and includes the kinematics of the arm support.
   Functions needed are should be included in the folder.
   
   Script by erick nunez
   created: 1/24/2019

*/

// Libraries to add /////////////////////////////////////////////////////////////////////////////////
#include <DynamixelSDK.h>
#include <PID_v1.h>

// OptoForce variables ////////////////////////////////////////////////////////////////////////////// 
#define xSensitivity 16.45
#define ySensitivity 18.42
#define zSensitivity 1.59
int16_t xRaw, yRaw, zRaw;
float  Fx, Fy, Fz, FxRaw, FyRaw, FzRaw;
float  xCal = 0, yCal = 0, zCal = 0;

// Encoder Variables ////////////////////////////////////////////////////////////////////////////////
#define encoderPinA  9
#define encoderPinB 10
int encoderCounter = 0;

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
#define ELBOW_MIN_POS     1739
#define ELBOW_MAX_POS     3774
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

// Linear Actuator Variables ////////////////////////////////////////////////////////////////////////
#define ACTUATOR_PIN_A    2
#define ACTUATOR_PIN_B    3
#define ACTUATOR_PWM_PIN  8
#define ACTUATOR_EN_PIN   0
#define ACTUATOR_CS_PIN   1
int Kp = 10, Ki = 0, Kd = 0;
double inputPID = 0, outputPID = 0, setPointPID = 0;
PID actuatorPID(&inputPID, &outputPID, &setPointPID, Kp, Ki, Kd, DIRECT); 
double goalPoint, maxHeight, minHeight;

// Admitance Control Variables //////////////////////////////////////////////////////////////////////
float xGoalPosSI, xGoalVelSI, yGoalPosSI, yGoalVelSI, zGoalPosSI, zGoalVelSI;
float xPresPosSI, xPresVelSI, yPresPosSI, yPresVelSI, zPresPosSI, zPresVelSI;
#define TIME         0.01 // loop time
#define MASS         0.5
#define DAMPING      25
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
  /* Encoder Pin Setup */
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), isEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), isEncoderB, CHANGE);
  delay(100);
  /* Linear Actuator Pin Setup */
  pinMode(ACTUATOR_PIN_A, OUTPUT);
  pinMode(ACTUATOR_PIN_B, OUTPUT);
  pinMode(ACTUATOR_PWM_PIN, OUTPUT);
  pinMode(ACTUATOR_CS_PIN, OUTPUT);
  pinMode(ACTUATOR_EN_PIN, OUTPUT);
  digitalWrite(ACTUATOR_EN_PIN, HIGH);
  actuatorPID.SetMode(AUTOMATIC);
  actuatorPID.SetSampleTime(1);
  actuatorPID.SetOutputLimits(-255,255);
  delay(100);
  /* Dynamixel Setup */
  if (portHandler -> openPort()){
    Serial.println(".....Opened the Dynamixel Port.....");
  }
  if (portHandler->setBaudRate(BAUDRATE)){
    Serial.println(".....Set baudrate.....");
  }
  if (!dxlAbling(POSITION_CONTROL, !ENABLE)){
    Serial.println(".....Enabled motors.....");
  }
  /* Optoforce Serial Connection */
  Serial.println(".....Creating sensor port.....");
  Serial1.begin(BAUDRATE);
  delay(100);
  Serial.println(".....Configuring sensor.....");
  optoForceConfig();
  delay(100);
  Serial.println(".....Calibrating sensor.....");
  calibrateForceSensor(xCal, yCal, zCal);  
  delay(1000);
  Serial.println(".....leaving setup.....");
}

// Main loop function ///////////////////////////////////////////////////////////////////////////////////
void loop() {
  dynamixel::GroupSyncWrite syncWritePacket(portHandler, packetHandler, ADDRESS_PROFILE_VELOCITY, LEN_PROFILE_VELOCITY + LEN_GOAL_POSITION);
  dynamixel::GroupSyncRead  syncReadPacket(portHandler, packetHandler, ADDRESS_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION);
  addParamResult = syncReadPacket.addParam(ID_SHOULDER);
  addParamResult = syncReadPacket.addParam(ID_ELBOW);
  /* Actuator Calibration */
  actuatorCalibration();
  /* Main Loop */
  while(1){
    /* Starts reading the sensor */ 

    if(Serial.available()>0){
        String goal = Serial.readString();
        goalPoint = goal.toInt();
    }
    
    preTime = millis();
    
    
    singleOptoForceRead(xRaw, yRaw, zRaw, FxRaw, FyRaw, FzRaw);
    readPresentPacket(syncReadPacket, presVelElbow, presPosElbow, presVelShoulder, presPosShoulder);
    sensorOrientation(FxRaw, FyRaw, presPosElbow, presPosShoulder, Fx, Fy, presElbowAng, presShoulderAng);
    forwardKine();
    admittanceControl(Fx, xPresPosSI, xPresVelSI, xGoalPosSI, xGoalVelSI, Fy, yPresPosSI, yPresVelSI, yGoalPosSI, yGoalVelSI);
    inverseKine();
  
    if ((goalPosElbow <= ELBOW_MAX_POS && goalPosElbow >= ELBOW_MIN_POS) & (goalPosShoulder <= SHOULDER_MAX_POS && goalPosShoulder >= SHOULDER_MIN_POS)){
      if ((goalVelElbow <= ELBOW_MAX_VEL && goalVelElbow >= ELBOW_MIN_VEL) & (goalVelShoulder <= SHOULDER_MAX_VEL && goalVelShoulder >= SHOULDER_MIN_VEL)){
        goalReturn = writeGoalPacket(syncWritePacket, goalVelElbow, goalPosElbow, goalVelShoulder, goalPosShoulder);
        if(goalPoint <= maxHeight && goalPoint >= minHeight){
          actuatorControl(goalPoint);
        }
      }
    }
  
    postTime = millis();
    
    noInterrupts();
    
    //delay(10-(postTime-preTime));

    Serial.print(encoderCounter); Serial.print("\t");
    Serial.print(inputPID); Serial.print("\t");
    Serial.print(setPointPID); Serial.print("\t");
    Serial.print(outputPID); Serial.print("\t");
    
    //Serial.print(FxRaw); Serial.print("\t");
    //Serial.print(FyRaw); Serial.print("\t");
  
    //Serial.print(Fx); Serial.print("\t");
    //Serial.print(Fy); Serial.print("\t");
  
    //Serial.print(presElbowAng); Serial.print("\t"); 
    //Serial.print(presShoulderAng); Serial.print("\t");
  
    //Serial.print(presVelElbow); Serial.print("\t"); 
    //Serial.print(presVelShoulder); Serial.print("\t");
  
    //Serial.print(presPosElbow); Serial.print("\t"); 
    //Serial.print(presPosShoulder); Serial.print("\t");
  
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
  
    //Serial.print(goalVelElbow); Serial.print("\t"); 
    //Serial.print(goalVelShoulder); Serial.print("\t");
  
    //Serial.print(goalPosElbow); Serial.print("\t");  
    //Serial.print(goalPosShoulder); Serial.print("\t");
  
    Serial.print(postTime-preTime); Serial.print("\t"); 
    Serial.print(goalReturn); Serial.print("\t");  
    Serial.print("\n");
  }
}
