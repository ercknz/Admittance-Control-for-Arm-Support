/* This class establishes the filter object for the force sensor.
   It uses an exponential filter in the form of:
   y(n) = w*x(n) + (1-w)*y(n-1)
   where: y(n) is the output value
          w is the filter weight between
          x(n) is the input value
          y(n-1) is the last output
          
   Created 1/28/2020
   Script by erick nunez
*/

class forceFilter {
  float weight;
  forceStruct currentForce;
  
  public:
  forceFilter(float initialValue, float filterWeight){
    currentForce.X = initialValue;
    currentForce.Y = initialValue;
    currentForce.Z = initialValue;
    weight = filterWeight;
  }
  forceStruct Update(forceStruct inputForce){
    forceStruct outputForce;
    outputForce.X = weight*inputForce.X + (1 - weight)*currentForce.X;
    outputForce.Y = weight*inputForce.Y + (1 - weight)*currentForce.Y;
    outputForce.Z = weight*inputForce.Z + (1 - weight)*currentForce.Z;
    currentForce = outputForce;
    return outputForce;
  }
  forceStruct GetCurrent(){
    return currentForce;
  }
};

class singleFilter {
  float weight;
  float currentValue;
  
  public:
  singleFilter(float initialValue, float filterWeight){
    currentValue = initialValue;
    weight = filterWeight;
  }
  float Update(float inputValue){
    float outputValue = weight*inputValue + (1 - weight)*currentValue;
    currentValue = outputValue;
    return outputValue;
  }
  float GetCurrent(){
    return currentValue;
  }
  void SetCurrent(float newCurrent){
    currentValue = newCurrent;
  }
};
