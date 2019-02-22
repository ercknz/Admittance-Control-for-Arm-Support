/* This function is used to reorient the X and Y forces with respect to the ground reference frame. 
   This function takes in the X and Y forces as well as the angles of the shoulder and elbow.
   
   Created 1/24/2019
   Script by erick nunez
*/

void sensorOrientation(){
  presElbowAng     = (presPosElbow - ELBOW_MIN_POS)    * DEGREES_PER_COUNT * (PI/180);
  presShoulderAng  = (presPosShoulder - SHOULDER_MIN_POS) * DEGREES_PER_COUNT * (PI/180);
  Fx = FxRaw * (cos(presShoulderAng + presElbowAng)) + FyRaw * (-sin(presShoulderAng + presElbowAng));
  Fy = FxRaw * (sin(presShoulderAng + presElbowAng)) + FyRaw * ( cos(presShoulderAng + presElbowAng));
}
