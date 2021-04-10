/* 

Body controlling a mass/spring physical model
By Helmer Nuijens

Using: 
- MiPhysics by .....
- Motion capture data import class by stefanG, see Read_Mocap tab

To do:
- X Get right velocity, acceleration and jerk data
- X Import camera lib (delete current camera)
- X Calculate box size/ extensiveness
- Calculate Laban descriptors
- Import miPhysics
- Create + interact with
- Control the physical parameters
- experiment for right mapping
- create more interactions

*/

//---Libraries---//
import peasy.*;                              // Library allowing camera movements

//---Classes---//
PeasyCam cam;                                // Camera
MocapInstance mocapInstance;                 // Mocap instance, see Read_Mocap

//---Variables---//
int currentFrame = 0;                        // Current frame 
float deltaT;                                // Frame time
PVector centerOfMass;                        // body center of mass
float extensiveness;                         // Body extensiveness
float[] jointWeight = new float[38];         // 38 joints in total

void setup()
{
  //---Display---//
  //fullScreen(P3D,1);
  size(800, 600, P3D);                       // 3D environment
  cam = new PeasyCam(this,150);              // Starting point camera
  cam.setMinimumDistance(50);                // Min camera distance 
  cam.setMaximumDistance(5500);              // Max camera distance 
  cam.rotateX(radians(200));                 // Rotate camera for orientation
  
  //---Motion captures---//
  Mocap mocap1 = new Mocap("05_18.bvh");     // Load motion capture data:               
  // Constructor:
  mocapInstance = new MocapInstance(mocap1,0,new float[] {0.,0.,0.},      
                                              new float[] {0.,0.,0.},1.,
                                              color(255, 0, 200),2);
                                              
  //---Frame Rate---//
  float frmRate = findFrameRate(mocapInstance);
  frameRate(frmRate);
  deltaT = 1/frmRate;
  
  //---Initialize joint weights---//
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

void draw()
{
  background(0, 0, 0); 
  drawGroundPlane(50);
  
  mocapInstance.drawMocap();
  
  currentFrame = mocapInstance.currentFrame;
  centerOfMass = getCenterOfMass();
  translate(centerOfMass.x,centerOfMass.y,centerOfMass.z);
  extensiveness = getExtensiveness();
  sphere(extensiveness/100.);
  
     // print("Vel ");
     // print(getVelScalar(0));
     // print(" - Acc: ");
     // print(getAccScalar(0));
     // print(" - Jerk:  ");
     // println(getJerkScalar(0));
     println(getExtensiveness());
}
