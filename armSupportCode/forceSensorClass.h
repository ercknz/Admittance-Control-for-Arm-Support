/* This class establishes the force sensor object.
   This class includes the raw readings, the rotated global forces and the filtered forces.
   It uses an exponential filter in the form of:
   y(n) = w*x(n) + (1-w)*y(n-1)
   where: y(n) is the output value
          w is the filter weight between
          x(n) is the input value
          y(n-1) is the last output

   Created 1/28/2020
   Script by erick nunez
*/

class forceSensor {
    float filterWeight;
    forceStruct lastFiltForce;
    forceStruct lastRawForce;
    forceStruct lastGlobalForce;
    forceStruct currentFiltForce;
    forceStruct currentRawForce;
    forceStruct currentGlobalForce;
    float threshold = 20.0;

  public:
    forceSensor(float weight, float initialValue = 0.1) {
      currentRawForce.X,     currentRawForce.Y,     currentRawForce.Z    = initialValue;
      currentGlobalForce.X,  currentGlobalForce.Y,  currentGlobalForce.Z = initialValue;
      currentFiltForce.X,    currentFiltForce.Y,    currentFiltForce.Z   = initialValue;
      filterWeight = weight;
    }

    void Update(jointSpace presentQ) {
      /* save previous readings */
      lastRawForce    = currentRawForce;
      lastGlobalForce = currentGlobalForce;
      lastFiltForce   = currentFiltForce;
      /* Read force sensor */
      currentRawForce = singleOptoForceRead(xCal, yCal, zCal);
      /* check force sensor readings */
      if ((currentRawForce.X != 0.0) && (abs(currentRawForce.X / lastRawForce.X) > threshold)) currentRawForce.X = lastRawForce.X;
      if ((currentRawForce.Y != 0.0) && (abs(currentRawForce.Y / lastRawForce.Y) > threshold)) currentRawForce.Y = lastRawForce.Y;
      if ((currentRawForce.Z != 0.0) && (abs(currentRawForce.Z / lastRawForce.Z) > threshold)) currentRawForce.Z = lastRawForce.Z;
      /* reorient force sensor readings */
      currentGlobalForce = sensorOrientation(currentRawForce, presentQ);
      /* filter force sensor readings*/
      currentFiltForce.X = filterWeight * currentGlobalForce.X + (1 - filterWeight) * lastFiltForce.X;
      currentFiltForce.Y = filterWeight * currentGlobalForce.Y + (1 - filterWeight) * lastFiltForce.Y;
      currentFiltForce.Z = filterWeight * currentGlobalForce.Z + (1 - filterWeight) * lastFiltForce.Z;
    }

    forceStruct GetCurrent() {
      return lastFiltForce;
    }
};

forceStruct Orientation(forceStruct rawF, jointSpace pres) {
  forceStruct globalF;
  // [GlobalForce] = Rz(q1+q4) * Rz(pi/2) * Ry(pi) * [rawForce]
  globalF.X = rawF.X * ( sin(pres.q1 + pres.q4)) + rawF.Y * (-cos(pres.q1 + pres.q4));
  globalF.Y = rawF.X * (-cos(pres.q1 + pres.q4)) + rawF.Y * (-sin(pres.q1 + pres.q4));
  globalF.Z = -rawF.Z;
  return globalF;
}
