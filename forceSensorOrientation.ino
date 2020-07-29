/* This function is used to reorient the X and Y forces with respect to the ground reference frame. 
   This function takes in the X and Y forces as well as the angles of the shoulder and elbow.
   
   Created 1/24/2019
   Script by erick nunez
*/

forceStruct sensorOrientation(forceStruct rawF, jointSpace pres){
  forceStruct globalF;

  // [GlobalForce] = Rz(q1+q4) * Rz(pi/2) * Ry(pi) * [rawForce]
  globalF.X = rawF.X * ( sin(pres.q1 + pres.q4)) + rawF.Y * (-cos(pres.q1 + pres.q4));
  globalF.Y = rawF.X * (-cos(pres.q1 + pres.q4)) + rawF.Y * (-sin(pres.q1 + pres.q4));
  globalF.Z = -rawF.Z;

  return globalF;
}
