/* These functions are used to solve the forward or inverse kinematics of the system in X and Y directions.
   forwardKine() takes the motor counts and outputs the position(SI) for the system.
   inverseKine() takes the position(SI) and outputs the motor counts for the new position.

   Created 1/24/2019
   Script by erick nunez
*/

/******************** Arm Support Forward Kinematics Function ************************************************/
modelSpace forwardKine(jointSpace Q) {
  /* motor counts/speed --> forwardKine() --> position/velocity(SI) */
  Serial.println("fKine");
  modelSpace M;

  // Compute the XY positions from angles
  M.x = (A1_LINK + A2_LINK) * cos(Q.q1) + L1_LINK * cos(Q.q1) * cos(Q.q2) + LINK_OFFSET * sin(Q.q1 + Q.q4) + L2_LINK * cos(Q.q1 + Q.q4);
  M.y = (A1_LINK + A2_LINK) * sin(Q.q1) + L1_LINK * sin(Q.q1) * cos(Q.q2) - LINK_OFFSET * cos(Q.q1 + Q.q4) + L2_LINK * sin(Q.q1 + Q.q4);
  M.z = L1_LINK * sin(Q.q2);

  // Multiply velocities with Jacobian Matrix to find the XY velocities
  float J11 = - (A1_LINK + A2_LINK) * sin(Q.q1) - L1_LINK * sin(Q.q1) * cos(Q.q2) + LINK_OFFSET * cos(Q.q1 + Q.q4) - L2_LINK * sin(Q.q1 + Q.q4);
  float J12 = - L1_LINK * cos(Q.q1) * sin(Q.q2);
  float J13 = LINK_OFFSET * cos(Q.q1 + Q.q4) - L2_LINK * sin(Q.q1 + Q.q4);
  float J21 = (A1_LINK + A2_LINK) * cos(Q.q1) + L1_LINK * cos(Q.q1) * cos(Q.q2) + LINK_OFFSET * sin(Q.q1 + Q.q4) + L2_LINK * cos(Q.q1 + Q.q4);
  float J22 = - L1_LINK * sin(Q.q1) * sin(Q.q2);
  float J23 = LINK_OFFSET * sin(Q.q1 + Q.q4) + L2_LINK * cos(Q.q1 + Q.q4);
  float J32 = L1_LINK * cos(Q.q2);  /* and J31 = J33 = 0 */
  M.xDot = Q.q1Dot * J11 + Q.q2Dot * J12 + Q.q4Dot * J13;
  M.yDot = Q.q1Dot * J21 + Q.q2Dot * J22 + Q.q4Dot * J23;
  M.zDot = Q.q2Dot * J32;

  return M;
}

/******************** Arm Support Inverse Kinematics function ************************************************/
jointSpace inverseKine(jointSpace pres, modelSpace &M) {
  Serial.println("iKine");
  Serial.print("X: "); Serial.println(M.x);
  Serial.print("Y: "); Serial.println(M.y);
  /* position/velocity(SI) --> inverseKine() --> motor counts/speed */
  jointSpace Q;
  Serial.print("Q1: "); Serial.println(Q.q1);
  Serial.print("Q4: "); Serial.println(Q.q4);

  /* JointSpace Limits */
  static float Q1_MIN    = SHOULDER_MIN_POS * DEGREES_PER_COUNT * (PI / 180);
  static float Q1_MAX    = SHOULDER_MAX_POS * DEGREES_PER_COUNT * (PI / 180);
  static float Q4_MIN    = 0.0;
  static float Q4_MAX    = (ELBOW_MAX_POS - ELBOW_MIN_POS) * DEGREES_PER_COUNT * (PI / 180);
  static float Q2_LIMIT  = (ELEVATION_MAX_POS - ELEVATION_CENTER) * DEGREES_PER_COUNT * (PI / 180.0) * (1 / ELEVATION_RATIO);
  static float INNER_DIA = A1_LINK + L1_LINK + A2_LINK - L2_LINK;

  //  Serial.println(Q1_MIN); Serial.println(Q1_MAX);

  /* Checks if X and Y are both 0 */
  if ((abs(M.x) < 0.001) && (abs(M.y) < 0.001)) {
    Q.q1 = pres.q1;
    Q.q4 = Q4_MAX;
    M = forwardKine(Q);
  }

  /* Check Z limits */
  static float Z_LIMIT = L1_LINK * sin(Q2_LIMIT);
  if (M.z >  Z_LIMIT) M.z =  Z_LIMIT;
  if (M.z < -Z_LIMIT) M.z = -Z_LIMIT;

  /* Find variables based on Z */
  Q.q2 = asin(M.z / L1_LINK);
  float L1_XY = sqrt(pow(L1_LINK, 2) - pow(M.z, 2));
  Serial.print("Q2: "); Serial.println(Q.q2);
  Serial.print("L1: "); Serial.println(L1_XY);

  /* Checks walls */
  float OUTER_DIA = A1_LINK + L1_XY + A2_LINK + H_OF_L2;
  float R = sqrt(pow(M.x, 2) + pow(M.y, 2));
  float alpha = atan2(M.y, M.x);
  if (alpha < 0) alpha += 2 * PI;
  if (R < INNER_DIA) {
    M.x = INNER_DIA * cos(alpha);
    M.y = INNER_DIA * sin(alpha);
    R = INNER_DIA;
  }
  if (R > OUTER_DIA) {
    M.x = OUTER_DIA * cos(alpha);
    M.y = OUTER_DIA * sin(alpha);
    R = OUTER_DIA;
  }
  Serial.print("alpha: "); Serial.println(alpha);

  /* Finds and checks Elbow Angle */
  float gamma = acos((pow((A1_LINK + L1_XY + A2_LINK), 2) + pow(H_OF_L2, 2) - pow(M.x, 2) - pow(M.y, 2)) / (2 * H_OF_L2 * (A1_LINK + L1_XY + A2_LINK)));
  Serial.print("gamma: "); Serial.println(gamma);

  Q.q4 = PI - gamma;
  if (Q.q4 < Q4_MIN) Q.q4 = Q4_MIN;
  if (Q.q4 > Q4_MAX) Q.q4 = Q4_MAX;

  /* Finds and checks shoulder angle */
  float beta = asin((H_OF_L2 * sin(gamma)) / R);
  Q.q1 = alpha - beta;
  if (Q.q1 < Q1_MIN) Q.q1 = Q1_MIN;
  if (Q.q1 > Q1_MAX) Q.q1 = Q1_MAX;
  Serial.print("beta: "); Serial.println(beta);

  /* Check for nans */
  if (isnan(Q.q1)) Q.q1 = pres.q1;
  if (isnan(Q.q4)) Q.q4 = pres.q4;

  Serial.print("Q1: "); Serial.println(Q.q1);
  Serial.print("Q4: "); Serial.println(Q.q4);

  /* Checks XYZ */
  modelSpace checkM = forwardKine(Q);
  if ((abs(M.x - checkM.x) > 0.001) || (abs(M.y - checkM.y) > 0.001) || (abs(M.y - checkM.y) > 0.001)) {
    M = checkM;
  }

  Serial.print("aftercCheckQ1: "); Serial.println(Q.q1);
  Serial.print("afterCheckQ4: "); Serial.println(Q.q4);

  /* Solve for joint angular velocities (psuedo inverse Jacobian) */
  float detJ = (-L1_LINK * sin(Q.q1) - L2_LINK * sin(Q.q1 + Q.q4)) * (L2_LINK * cos(Q.q1 + Q.q4)) - (-L2_LINK * sin(Q.q1 + Q.q4)) * (L1_LINK * cos(Q.q1) + L2_LINK * cos(Q.q1 + Q.q4));
  Q.q1Dot = (M.xDot * (L2_LINK * cos(Q.q1 + Q.q4)) + M.yDot * (L2_LINK * sin(Q.q1 + Q.q4))) / detJ;
  Q.q2Dot = M.zDot / (L1_LINK * sqrt(1 - pow((M.z / L1_LINK), 2)));
  Q.q4Dot = -(M.xDot * (-L1_LINK * cos(Q.q1) - L2_LINK * cos(Q.q1 + Q.q4)) + M.yDot * (-L1_LINK * sin(Q.q1) - L2_LINK * sin(Q.q1 + Q.q4))) / detJ;

  /* Convert to Counts */
  Q.q1Cts    = Q.q1 * (180.0 / PI) / DEGREES_PER_COUNT;
  Q.q2Cts    = ELEVATION_CENTER - (Q.q2 * ELEVATION_RATIO * (180.0 / PI) / DEGREES_PER_COUNT);
  Q.q4Cts    = ELBOW_MIN_POS + Q.q4 * (180.0 / PI) / DEGREES_PER_COUNT;
  Q.q1DotCts = abs(Q.q1Dot * (60.0 / (2.0 * PI)) / RPM_PER_COUNT);
  Q.q2DotCts = abs(Q.q2Dot * (60.0 / (2.0 * PI)) / RPM_PER_COUNT) * ELEVATION_RATIO;
  Q.q4DotCts = abs(Q.q4Dot * (60.0 / (2.0 * PI)) / RPM_PER_COUNT);

  return Q;
}
