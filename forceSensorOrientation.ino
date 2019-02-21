/* This function is used to reorient the X and Y forces with respect to the ground reference frame. 
   This function takes in the X and Y forces as well as the angles of the shoulder and elbow.
   
   Created 1/24/2019
   Script by erick nunez
*/

void sensorOrientation(){
  Fx = FxRaw * (cos(shoulderAngle + elbowAngle)) + FyRaw * (-sin(shoulderAngle + elbowAngle));
  Fy = FxRaw * (sin(shoulderAngle + elbowAngle)) + FyRaw * ( cos(shoulderAngle + elbowAngle));
}
