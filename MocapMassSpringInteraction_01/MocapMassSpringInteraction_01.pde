/* 

Body controlling physical model
By Helmer Nuijens

Using: 
- MiPhysics by .....
- Motion capture data import class by stefanG, see Read_Mocap tab

To do:
- Get right velocity, acceleration and jerk data
- X Import camera lib (delete current camera)
- Calculate box size
- Calculate Laban descriptors
- Import miPhysics
- Create + interact with
- Control the physical parameters
- experiment for right mapping
- create more interactions\

*/

// Libraries:
import peasy.*;                // Library allowing camera movements


// Classes: 
PeasyCam cam; //camera
MocapInstance mocapInstance;

float rX, rZ, vX, vZ;
int currentFrame = 0;

float deltaT; 

void setup()
{
 //--- Display ---  //
  fullScreen(P3D,1);                                       // 3D environment
  cam = new PeasyCam(this,150);                            // Starting point camera
  cam.setMinimumDistance(50);                              // Min camera distance 
  cam.setMaximumDistance(5500);                            // Max camera distance 
  cam.rotateX(radians(200));                               // Rotate camera for orientation
  
  //--- Motion captures --- //
  Mocap mocap1 = new Mocap("05_20.bvh");
  
  //--- Drawing Mocap ---//
  mocapInstance = new MocapInstance(mocap1,0,new float[] {0.,0.,0.},
                                              new float[] {0.,0.,0.},1.,
                                              color(255, 0, 200),2);
                                              
  //--- Frame Rate ---
  float frmRate = findFrameRate(mocapInstance);
  frameRate(frmRate);
  deltaT = 1/frmRate;
}

void draw()
{
  background(0, 0, 0); 
  drawGroundPlane(50);
  
  mocapInstance.drawMocap();
  
  currentFrame = mocapInstance.currentFrame;

  translate(getPosVec(0,0).x,getPosVec(0,0).y,getPosVec(0,0).z);

  sphere(getVelScalar(0)/10.);
  
      print("Vel ");
      print(getVelScalar(0));
      print(" - Acc: ");
      print(getAccScalar(0));
      print(" - Jerk:  ");
      println(getJerkScalar(0));

}
