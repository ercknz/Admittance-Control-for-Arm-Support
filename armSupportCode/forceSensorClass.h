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

  public:
    forceSensor(float weight, float initialValue = 0.001) {
      lastFiltForce.X = initialValue;
      lastFiltForce.Y = initialValue;
      lastFiltForce.Z = initialValue;
      filterWeight = weight;
    }

    forceStruct Orientation(jointSpace pres) {
      forceStruct globalF;
      // [GlobalForce] = Rz(q1+q4) * Rz(pi/2) * Ry(pi) * [rawForce]
      globalF.X = rawF.X * ( sin(pres.q1 + pres.q4)) + rawF.Y * (-cos(pres.q1 + pres.q4));
      globalF.Y = rawF.X * (-cos(pres.q1 + pres.q4)) + rawF.Y * (-sin(pres.q1 + pres.q4));
      globalF.Z = -rawF.Z;
      return globalF;
    }

    forceStruct Update(jointSpace presentQ) {
      /* Read force sensor */
      currentRawForce = singleOptoForceRead(xCal, yCal, zCal);
      /* cheack force sensor readings */
      if (currentRawForce.X > 20.0*lastRawForce) {}
      if (currentRawForce.Y > 20.0*lastRawForce) {}
      if (currentRawForce.Y > 20.0*lastRawForce) {}
      /* reorient force sensor readings */
      currentGlobalForce = sensorOrientation(currentRawForce, presentQ);
      /* filter force sensor readings*/
      currentFiltForce.X = filterWeight * currentGlobalForce.X + (1 - filterWeight) * lastFiltForce.X;
      currentFiltForce.Y = filterWeight * currentGlobalForce.Y + (1 - filterWeight) * lastFiltForce.Y;
      currentFiltForce.Z = filterWeight * currentGlobalForce.Z + (1 - filterWeight) * lastFiltForce.Z;
      return outputForce;
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
