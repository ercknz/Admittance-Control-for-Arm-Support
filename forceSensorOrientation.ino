/* This function is used to reorient the X and Y forces with respect to the ground reference frame. 
   This function takes in the X and Y forces as well as the angles of the shoulder and elbow.
   
   Created 1/24/2019
   Script by erick nunez
*/

void sensorOrientation(float alpha, float beta, double xForceRaw, double yForceRaw, double &xForce, double &yForce){
  xForce = xForceRaw * (cos(alpha + beta)) + yForceRaw * (-sin(alpha + beta));
  yForce = xForceRaw * (sin(alpha + beta)) + yForceRaw * ( cos(alpha + beta));
}
