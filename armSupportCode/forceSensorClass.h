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
  float weight;
  forceStruct currentFiltForce;
  forceStruct currentRawForce;
  forceStruct currentGlobalForce;
  
  public:
  forceSensor(float initialValue, float filterWeight){
    lastFiltForce.X = initialValue;
    lastFiltForce.Y = initialValue;
    lastFiltForce.Z = initialValue;
    weight = filterWeight;
  }
  
  forceStruct Orientation(forceStruct rawF, jointSpace pres){
  forceStruct globalF;
  // [GlobalForce] = Rz(q1+q4) * Rz(pi/2) * Ry(pi) * [rawForce]
  globalF.X = rawF.X * ( sin(pres.q1 + pres.q4)) + rawF.Y * (-cos(pres.q1 + pres.q4));
  globalF.Y = rawF.X * (-cos(pres.q1 + pres.q4)) + rawF.Y * (-sin(pres.q1 + pres.q4));
  globalF.Z = -rawF.Z;
  return globalF;
  }
  
  forceStruct Update(forceStruct inputForce){
    forceStruct outputForce;
    outputForce.X = weight*inputForce.X + (1 - weight)*lastFiltForce.X;
    outputForce.Y = weight*inputForce.Y + (1 - weight)*lastFiltForce.Y;
    outputForce.Z = weight*inputForce.Z + (1 - weight)*lastFiltForce.Z;
    lastFiltForce = outputForce;
    return outputForce;
  }
  
  forceStruct GetCurrent(){
    return lastFiltForce;
  }
};
