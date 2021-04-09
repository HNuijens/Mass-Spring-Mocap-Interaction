/*

Movement Analysis functions
Written by Helmer Nuijens 

-In each function 'j' stands for joint number
-In the GetPosVec function relFrame is the frame relative to the current frame number
*/

// Get Position
PVector getPosVec(int j, int relFrame)   
{
  PVector vector = new PVector();
  vector.x = mocap.joints.get(j).position.get((currentFrame + relFrame + mocap.frameNumber) % mocap.frameNumber).x;
  vector.y = mocap.joints.get(j).position.get((currentFrame + relFrame + mocap.frameNumber) % mocap.frameNumber).y;
  vector.z = mocap.joints.get(j).position.get((currentFrame + relFrame + mocap.frameNumber) % mocap.frameNumber).z;
  return vector;
}

// Calculate Velocity
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
  float velScalar = sqrt(pow(velVec.x,2)+ pow(velVec.y,2) + pow(velVec.z,2));
  return velScalar;
}

// Calculate Acceleration
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

// Calculate Jerk
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
  float jerkScalar = sqrt(pow(jerkVec.x,2)+ pow(jerkVec.y,2) + pow(jerkVec.z,2));
  return jerkScalar;
}
