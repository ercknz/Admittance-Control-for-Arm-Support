/* These functions are used to solve the forward or inverse kinematics of the system in X and Y directions.
   forwardKine() takes the motor counts and outputs the position(SI) for the system.
   inverseKine() takes the position(SI) and outputs the motor counts for the new position.
   
   Created 1/24/2019
   Script by erick nunez
*/

modelSpace forwardKine(jointSpace Q){
  // motor counts/speed --> forwardKine() --> position/velocity(SI)
  modelSpace M;
  // Compute the XY positions from angles 
  M.x = SHOULDER_ELBOW_LINK * cos(Q.q1) + ELBOW_SENSOR_LINK * cos(Q.q1 + Q.q2);
  M.y = SHOULDER_ELBOW_LINK * sin(Q.q1) + ELBOW_SENSOR_LINK * sin(Q.q1 + Q.q2);
  
  // Multiply velocities with Jacobian Matrix to find the XY velocities
  M.xDot = Q.q1Dot * (-SHOULDER_ELBOW_LINK * sin(Q.q1) - ELBOW_SENSOR_LINK * sin(Q.q1 + Q.q2)) + Q.q2Dot * (-ELBOW_SENSOR_LINK * sin(Q.q1 + Q.q2));
  M.yDot = Q.q1Dot * ( SHOULDER_ELBOW_LINK * cos(Q.q1) + ELBOW_SENSOR_LINK * cos(Q.q1 + Q.q2)) + Q.q2Dot * ( ELBOW_SENSOR_LINK * cos(Q.q1 + Q.q2));
  
  return M;
}

jointSpace inverseKine(modelSpace M){
  // position/velocity(SI) --> inverseKine() --> motor counts/speed
  jointSpace Q;
  // Solving for joint angles
  Q.q2 = acos((pow(M.x,2) + pow(M.y,2) - pow(SHOULDER_ELBOW_LINK,2) - pow(ELBOW_SENSOR_LINK,2))/(2.0 * SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK));
  if (M.y < 0){
    Q.q1 = atan2(M.y, M.x) - atan2((ELBOW_SENSOR_LINK * sin(Q.q2)),(SHOULDER_ELBOW_LINK + ELBOW_SENSOR_LINK * cos(Q.q2))) + 2.0*PI;
  } else {
    Q.q1 = atan2(M.y, M.x) - atan2((ELBOW_SENSOR_LINK * sin(Q.q2)),(SHOULDER_ELBOW_LINK + ELBOW_SENSOR_LINK * cos(Q.q2)));
  }
  Q.q2Dot = (M.xDot * (ELBOW_SENSOR_LINK * cos(Q.q1 + Q.q2)) + M.yDot * (ELBOW_SENSOR_LINK * sin(Q.q1 + Q.q2)))/(SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK * sin(Q.q2));
  Q.q1Dot = (M.xDot * (SHOULDER_ELBOW_LINK * cos(Q.q1) - ELBOW_SENSOR_LINK * cos(Q.q1 + Q.q2)) + M.yDot * (-SHOULDER_ELBOW_LINK * sin(Q.q1) - ELBOW_SENSOR_LINK * sin(Q.q1 + Q.q2)))/(SHOULDER_ELBOW_LINK * ELBOW_SENSOR_LINK * sin(Q.q2));

  return Q;
}
