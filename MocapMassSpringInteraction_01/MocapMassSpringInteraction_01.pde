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
import miPhysics.Renderer.*;                 // Libraries for mass/spring interactions
import miPhysics.Engine.*;
import miPhysics.Engine.Sound.*;

//---Classes---//
PeasyCam cam;                                // Camera
MocapInstance mocapInstance;                 // Mocap, see Read_Mocap
PhysicsContext phys;                         // Physics
Observer3D listener;                         // Audio out
PosInput3D[] input = new PosInput3D[10];     // Excitation inputs
ModelRenderer renderer;                      // Rendering
miString string;                             // String
miPhyAudioClient audioStreamHandler;         // Audio

//---Variables---//
// Mocap
int currentFrame = 0;                        // Current frame 
float deltaT;                                // Frame time
PVector centerOfMass;                        // body center of mass
float extensiveness;                         // Body extensiveness
float[] jointWeight = new float[38];         // 38 joints in total
float scaling = 1./100;                       // object scaling mocap and model

// Input
int nInput = 10;                             // Number of input masses
int[] smoothing = new int[10];               // Input smoothing (sensitivity)
float inRadius = 1*scaling;                  // Radius of input masses
float[][] inputPos = new float[10][3];       // relative positioning of input masses

// Strings
float m = 1.f;                               // Mass
float k = 0.6f;                              // Stiffess
float z = 0.05f;                             // Damping
float dist = 0.1f;                           // Distance between individual masses
int nMass = 100;                             // Number of masses in string
float radius = 0.1;                          // Radius of masses

// Audio
float currAudio = 0;
float audioOut = 0;

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
  println(frmRate);
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

  //---Physical model setup---//
  Medium med = new Medium(0.00001, new Vect3D(0, -0.00, 0.0));  // Create new medium with friction and gravity
  phys = new PhysicsContext(44100, int(frmRate));               // Create context with right sample and frame rates
  
  // Create and tranform string
  string = new miString("string", med, nMass, radius, m, k, z, dist, 0.02, "2D");
  string.rotate(0, PI/2, 0);
  string.translate(-8,0,0);
  string.changeToFixedPoint("m_0");                            // Ground 1
  string.changeToFixedPoint(string.getLastMass());             // Ground 2
  
  PhyModel perc = new PhyModel("pluck", med);                  // Physical model
  
  //---Input setup---//
  centerOfMass = getCenterOfMass();
  extensiveness = getExtensiveness();
  // set relative positions
  setInputPos(0, extensiveness, 0,0,0);   // Middle input (center of mass)
  setInputPos(1, extensiveness, -2,0,0);  // 2nd input
  setInputPos(2, extensiveness, 2,0,0);   // 3rd input
  setInputPos(3, extensiveness, 0,-2,0);  // 4th input
  setInputPos(4, extensiveness, 0,2,0);   // 5th input
  setInputPos(5, extensiveness, 0,0,-2);  // 6th input
  setInputPos(6, extensiveness, 0,0,2);   // 7th input
  setInputPos(7, extensiveness, 2,2,0);   // 8th input
  setInputPos(8, extensiveness, -2,-2,0); // 9th input
  setInputPos(9, extensiveness, 2,2,2);   // 10th input

  for(int i = 0; i < nInput; i++)
  {
    smoothing[i] = 1000 + (int) random(10,10000); // Each input has a slightly different response 
    input[i] = perc.addMass("input"+i, new PosInput3D(inRadius, new Vect3D(centerOfMass.x + inputPos[i][0], centerOfMass.y + inputPos[i][1], centerOfMass.z + inputPos[i][2]), smoothing[i]));
  }
  smoothing[0] = 100; // Middle input mass more responsive than others
  input[0].setParam(param.RADIUS, 2./100); // Make center input slightly larger
  
  phys.mdl().addPhyModel(string);
  phys.mdl().addPhyModel(perc);
  
  phys.mdl().addInOut("listener1", new Observer3D(filterType.HIGH_PASS), phys.mdl().getPhyModel("string").getMass("m_10"));
  phys.colEngine().addCollision(string, perc, 0.0002, -0.001);   // Bowing settings
  //phys.colEngine().addCollision(string, perc, 0.05, 0.01);    // Plucking settings

  phys.init();
  
  // Visuals
  renderer = new ModelRenderer(this);  
  renderer.setZoomVector(100, 100, 100);  
  renderer.displayMasses(true);  
  
  renderer.setColor(massType.MASS2DPLANE, 255, 250, 200);
  renderer.setColor(interType.SPRINGDAMPER3D, 255, 70, 70, 255);
  renderer.setColor(massType.POSINPUT3D, 160, 220, 255);
  renderer.setStrainGradient(interType.SPRINGDAMPER3D, true, 0.1);
  renderer.setStrainColor(interType.SPRINGDAMPER3D, 255, 100, 200, 200);
  
  // Audio 
  audioStreamHandler = miPhyAudioClient.miPhyClassic(44100, 512, 0, 2, phys);
  audioStreamHandler.setListenerAxis(listenerAxis.Y);
  audioStreamHandler.setGain(0.3);
  audioStreamHandler.start();

}

void draw()
{
  noCursor();
  background(0, 0, 0); 
  directionalLight(126, 126, 126, 100, 0, -1);
  ambientLight(182, 182, 182);
  drawGroundPlane(50);
  //println(getPosVec(0,0));
  mocapInstance.drawMocap();
  
  currentFrame = mocapInstance.currentFrame;
  centerOfMass = getCenterOfMass();
  extensiveness = getExtensiveness(); 
   println(extensiveness);
  for(int i = 0; i < nInput; i++)
  {
    input[i].drivePosition(new Vect3D((centerOfMass.x + inputPos[i][0]*extensiveness)*scaling, (centerOfMass.y + inputPos[i][2]*extensiveness)*scaling, (centerOfMass.z + inputPos[i][2]*extensiveness)*scaling));
  }
  renderer.renderScene(phys);
  
     // print("Vel ");
     // print(getVelScalar(0));
     // print(" - Acc: ");
     // print(getAccScalar(0));
     // print(" - Jerk:  ");
     // println(getJerkScalar(0));
     //println(getExtensiveness());
}
