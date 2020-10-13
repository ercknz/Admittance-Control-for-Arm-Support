/* This establishes the force sensor functions and filter class.
   This class includes the last reading and the new filtered forces.
   It uses an exponential filter in the form of:
   y(n) = w*x(n) + (1-w)*y(n-1)
   where: y(n) is the output value
          w is the filter weight between
          x(n) is the input value
          y(n-1) is the last output

   Created 1/28/2020
   Script by erick nunez
*/

/******************** Force Sensor Exponential Filter Class ************************************************/
class forceFilter {
    float weight;
    forceStruct currentForce;

  public:
    forceFilter(float filterWeight, float initialValue = 0.0) {
      currentForce.X = initialValue;
      currentForce.Y = initialValue;
      currentForce.Z = initialValue;
      weight = filterWeight;
    }
    forceStruct Update(forceStruct inputForce) {
      forceStruct outputForce;
      outputForce.X = weight * inputForce.X + (1 - weight) * currentForce.X;
      outputForce.Y = weight * inputForce.Y + (1 - weight) * currentForce.Y;
      outputForce.Z = weight * inputForce.Z + (1 - weight) * currentForce.Z;
      currentForce = outputForce;
      return outputForce;
    }
    forceStruct GetCurrent() {
      return currentForce;
    }
};

/******************** Force Sensor Global Orientation Function ************************************************/
forceStruct sensorOrientation(forceStruct rawF, jointSpace pres) {
  forceStruct globalF;
  /* [GlobalForce] = Rz(q1+q4) * Rz(pi/2) * Ry(pi) * [rawForce] */
  globalF.X = rawF.X * ( sin(pres.q1 + pres.q4)) + rawF.Y * (-cos(pres.q1 + pres.q4));
  globalF.Y = rawF.X * (-cos(pres.q1 + pres.q4)) + rawF.Y * (-sin(pres.q1 + pres.q4));
  globalF.Z = -rawF.Z;
  return globalF;
}

/******************** Force Sensor readings Check Function ************************************************/
forceStruct forceCheck(forceStruct newForces, forceStruct lastForces, float threshold, float deltaT) {
  forceStruct checkedForces = newForces;
  if (abs((newForces.X - lastForces.X) / deltaT) > threshold) checkedForces.X = lastForces.X;
  if (abs((newForces.Y - lastForces.Y) / deltaT) > threshold) checkedForces.Y = lastForces.Y;
  if (abs((newForces.Z - lastForces.Z) / deltaT) > threshold) checkedForces.Z = lastForces.Z;
  return checkedForces;
}


