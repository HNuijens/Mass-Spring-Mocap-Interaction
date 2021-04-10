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
  return extension/200;
}

//-------------------------------------
// High level motion descriptors ------
//-------------------------------------

//-------------------------------------
// Visualization motion          ------
//-------------------------------------

void setInputPos(int nInput, float dist, float x, float y, float z)
{
 inputPos[nInput][0] = x * dist;
 inputPos[nInput][1] = y * dist;
 inputPos[nInput][2] = z * dist;
}
