/* This function is used to reorient the X and Y forces with respect to the ground reference frame. 
   This function takes in the X and Y forces as well as the angles of the shoulder and elbow.
   
   Created 1/24/2019
   Script by erick nunez
*/

forceStruct sensorOrientation(forceStruct rawF, jointSpace pres){
  forceStruct globalF;
  globalF.X = rawF.X * (cos(pres.q1 + pres.q2 + PI/2.0)) + rawF.Y * (-sin(pres.q1 + pres.q2 + PI/2.0));
  globalF.Y = rawF.X * (sin(pres.q1 + pres.q2 + PI/2.0)) + rawF.Y * ( cos(pres.q1 + pres.q2 + PI/2.0));
  globalF.Z = rawF.Z;

  return globalF;
}
