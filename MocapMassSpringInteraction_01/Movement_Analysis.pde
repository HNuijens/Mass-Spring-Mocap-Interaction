/*

Movement Analysis functions
Written by Helmer Nuijens 

first part describes low level descriptors
second part describes high level motion descriptors

-In each function 'j' stands for joint number

*/


//-------------------------------------
// Low level motion descriptors -------
//-------------------------------------

//--- Get Position  ---//
PVector getPosVec(int j, int relFrame)   //relFrame is the frame relative to the current frame number
{
  PVector vector = new PVector();
  vector.x = mocap.joints.get(j).position.get((currentFrame + relFrame + mocap.frameNumber) % mocap.frameNumber).x;
  vector.y = mocap.joints.get(j).position.get((currentFrame + relFrame + mocap.frameNumber) % mocap.frameNumber).y;
  vector.z = mocap.joints.get(j).position.get((currentFrame + relFrame + mocap.frameNumber) % mocap.frameNumber).z;
  return vector;
}

//---Calculate Velocity---//
PVector getVelVec(int j)
{
  PVector vector = new PVector();
  vector.x = (getPosVec(j,1).x - getPosVec(j,-1).x)/(2. * deltaT);
  vector.y = (getPosVec(j,1).y - getPosVec(j,-1).y)/(2. * deltaT);
  vector.z = (getPosVec(j,1).z - getPosVec(j,-1).z)/(2. * deltaT);
  return vector;
}

float getVelScalar(int j)
{
  PVector velVec = getVelVec(j);
  float velScalar = sqrt(pow(velVec.x,2) + pow(velVec.y,2) + pow(velVec.z,2));
  return velScalar;
}

//---Calculate Acceleration---//
PVector getAccVec(int j)
{
  PVector vector = new PVector();
  vector.x = (getPosVec(j,1).x - 2 * getPosVec(j,0).x + getPosVec(j,-1).x)/(deltaT * deltaT);
  vector.y = (getPosVec(j,1).y - 2 * getPosVec(j,0).y + getPosVec(j,-1).y)/(deltaT * deltaT);
  vector.z = (getPosVec(j,1).z - 2 * getPosVec(j,0).z + getPosVec(j,-1).z)/(deltaT * deltaT);
  return vector;
}

float getAccScalar(int j)
{
  PVector accVec = getAccVec(j);
  float accScalar = sqrt(pow(accVec.x,2)+ pow(accVec.y,2) + pow(accVec.z,2));
  return accScalar;
}

//---Calculate Jerk---//
PVector getJerkVec(int j)
{
  PVector vector = new PVector();
  vector.x = (getPosVec(j,2).x - 2 * getPosVec(j,1).x + 2 * getPosVec(j,-1).x - getPosVec(j,-2).x)/(2. * deltaT * deltaT * deltaT);
  vector.y = (getPosVec(j,2).y - 2 * getPosVec(j,1).y + 2 * getPosVec(j,-1).y - getPosVec(j,-2).y)/(2. * deltaT * deltaT * deltaT);
  vector.z = (getPosVec(j,2).z - 2 * getPosVec(j,1).z + 2 * getPosVec(j,-1).z - getPosVec(j,-2).z)/(2. * deltaT * deltaT * deltaT);
  return vector;
}

float getJerkScalar(int j)
{
  PVector jerkVec = getJerkVec(j);
  float jerkScalar = sqrt(pow(jerkVec.x,2) + pow(jerkVec.y,2) + pow(jerkVec.z,2));
  return jerkScalar;
}

//---Initialize joint weights---//
void setJointWeights()
{
  for(int j = 0; j < jointWeight.length; j++)
  {
    jointWeight[j] = 0.; // initializs first with zeros
    
  }
  
  jointWeight[2] = 0.1000;  // right hip 
  jointWeight[3] = 0.0465;  // right knee
  jointWeight[6] = 0.0145;  // right toe joint
  jointWeight[8] = 0.1000;  // left hip
  jointWeight[9] = 0.0465;  // left knee
  jointWeight[11] = 0.0145; // left toe joint
  jointWeight[14] = 0.4970; // mid back
  jointWeight[18] = 0.0810; // head
  jointWeight[21] = 0.0280; // right shoulder
  jointWeight[22] = 0.0160; // right elbow
  jointWeight[25] = 0.0060; // right hand index
  jointWeight[30] = 0.0280; // left shoulder
  jointWeight[31] = 0.0160; // left elbow
  jointWeight[34] = 0.0060; // left hand index
}

//---Calculate Center of Mass---//
PVector getCenterOfMass()
{
  PVector ctrOfMass = new PVector();
  // weighted average of joints:
   for(int j = 0; j < mocap.joints.size(); j++) { 
      ctrOfMass.x += jointWeight[j] * mocap.joints.get(j).position.get(currentFrame).x;
      ctrOfMass.y += jointWeight[j] * mocap.joints.get(j).position.get(currentFrame).y;
      ctrOfMass.z += jointWeight[j] * mocap.joints.get(j).position.get(currentFrame).z;
    }
  return ctrOfMass;
}

//---Calculate Extensiveness---//
float getExtensiveness()
{
  float extension = 0;
  PVector ctrOfMass = getCenterOfMass();
  // Summation of distance between joints and center of mass
  for(int j = 0; j < mocap.joints.size(); j++) { 
    extension += sqrt(pow(ctrOfMass.x - getPosVec(j,0).x,2) +  pow(ctrOfMass.y - getPosVec(j,0).y,2) + pow(ctrOfMass.z - getPosVec(j,0).z,2));
  }
  return extension/25;
}

//-------------------------------------
// High level motion descriptors ------
//-------------------------------------

//--- Setup arrays for storage---//
void setPeriod(int period)
{
  for(int t =0; t <period ; t++)
  {
    weightEffortList.append(0.);
  }
}

//---Calculate Weight Effort---//
float getWeightEffort()
{
  float energy = 0.;
  float weight = 0.;
  for(int j = 0; j < mocap.joints.size(); j++) { 
    energy = energy + (1./mocap.joints.size()) * pow(getVelScalar(j),2);
  }
  weightEffortList.set(idx, energy);
  
  // Return largest in time period
  for( int i = 0; i < weightEffortList.size(); i ++)
  {
   if(weightEffortList.get(i)>weight)
   {
     weight = weightEffortList.get(i);
   }
  }
  idx = (idx + 1) % period;  
  return weight; 
}



//-------------------------------------
// Visualization motion          ------
//-------------------------------------

void setInputPos(int input,float spread)
{
 
}
