/* 

Mocap Classes and functions written by StefanG 
From: https://openprocessing.org/sketch/78767 
Accessed on 29/03/2021

This code makes is possible to import and analyse .bvh motion capture files

Modifications by Helmer Nuijens:
  - Removed unused functions
*/

//-------------------------------------
// Classes ----------------------------
//-------------------------------------

Mocap mocap; // class used inside mocapInstance

class MocapInstance {
  
  int currentFrame, firstFrame, lastFrame;
  float[] translation, rotation;
  float scl, strkWgt;
  color clr;

  MocapInstance (Mocap mocap1, int startingFrame, 
                 float[] transl, float[] rot, 
                 float scl1, color clr1, float strkWgt1) {
    mocap = mocap1;
    currentFrame = startingFrame;
    firstFrame = startingFrame;
    lastFrame = startingFrame-1;    
    translation = transl;
    rotation = rot;
    scl = scl1;
    clr = clr1;
    strkWgt = strkWgt1;
  }

  void drawMocap() {
    pushMatrix();
    stroke(clr);
    strokeWeight(strkWgt);
    translate(translation[0], translation[1], translation[2]);
    rotateX(radians(rotation[0]));
    rotateY(radians(rotation[1]));
    rotateZ(radians(rotation[2]));
    scale(scl);

    for(Joint itJ : mocap.joints) {
      line(itJ.position.get(currentFrame).x, 
           itJ.position.get(currentFrame).y, 
           itJ.position.get(currentFrame).z, 
           itJ.parent.position.get(currentFrame).x, 
           itJ.parent.position.get(currentFrame).y, 
           itJ.parent.position.get(currentFrame).z);
    } 
    popMatrix();
    currentFrame = (currentFrame+1) % (mocap.frameNumber);
    if (currentFrame==lastFrame+1) currentFrame = firstFrame;
  }
  
}

class Mocap {

  float frmRate;
  int frameNumber;
  float frameTime;
  ArrayList<Joint> joints = new ArrayList<Joint>();

  Mocap (String fileName) {
       String[] lines = loadStrings(fileName);
    int readMotion = 0;
    int lineMotion = 0;
    Joint currentParent = new Joint();

    for (int i=0;i<lines.length;i++) {

      //--- Read hierarchy ---  
      String[] words = splitTokens(lines[i], " \t");

      //list joints, with parent
      if (words[0].equals("ROOT")||words[0].equals("JOINT")||words[0].equals("End")) {
        Joint joint = new Joint();
        joints.add(joint);
        if (words[0].equals("End")) {
          joint.name = "EndSite"+((Joint)joints.get(joints.size()-1)).name;
          joint.isEndSite = 1;
        }
        else joint.name = words[1];
        if (words[0].equals("ROOT")) {
          joint.isRoot = 1;
          currentParent = joint;
        }
        joint.parent = currentParent;
      }

      //find parent
      if (words[0].equals("{"))
        currentParent = (Joint)joints.get(joints.size()-1); 
      if (words[0].equals("}")) {
        currentParent = currentParent.parent;
      }

      //offset
      if (words[0].equals("OFFSET")) {
        joints.get(joints.size()-1).offset.x = float(words[1]);
        joints.get(joints.size()-1).offset.y = float(words[2]);
        joints.get(joints.size()-1).offset.z = float(words[3]);
      }

      //order of rotations
      if (words[0].equals("CHANNELS")) {
        joints.get(joints.size()-1).rotationChannels[0] = words[words.length-3];
        joints.get(joints.size()-1).rotationChannels[1] = words[words.length-2];
        joints.get(joints.size()-1).rotationChannels[2] = words[words.length-1];
      }

      if (words[0].equals("MOTION")) {
        readMotion = 1;
        lineMotion = i;
      }

      if (words[0].equals("Frames:"))
        frameNumber = int(words[1]);

      if (words[0].equals("Frame") && words[1].equals("Time:")) {
        frameTime = float(words[2]);
        frmRate = round(1000./frameTime)/1000.;
      }

      //--- Read motion, compute positions ---    
      if (readMotion==1 && i>lineMotion+2) {
        
        //motion data
        PVector RotRelativPos = new PVector();
        int iMotionData = 3;// number of data points read, skip root position       
        for (Joint itJ : joints) {
          if (itJ.isEndSite==0) {// skip end sites
            float[][] currentTransMat = {{1., 0., 0.},{0., 1., 0.},{0., 0., 1.}};
            //The transformation matrix is the (right-)product 
            //of transformations specified by CHANNELS
            for (int iC=0;iC<itJ.rotationChannels.length;iC++) {
              currentTransMat = multMat(currentTransMat, 
                                        makeTransMat(float(words[iMotionData]), 
                                                     itJ.rotationChannels[iC]));
              iMotionData++;
            }
            if (itJ.isRoot==1) {//root has no parent: 
                                //transformation matrix is read directly
              itJ.transMat = currentTransMat;
            } 
            else {//other joints: 
                  //transformation matrix is obtained by right-applying 
                  //the current transformation to the transMat of parent
              itJ.transMat = multMat(itJ.parent.transMat, currentTransMat);
            }
          } 

          //positions
          if (itJ.isRoot==1) {//root: position read directly + offset
            RotRelativPos.set(float(words[0]), float(words[1]), float(words[2]));
            RotRelativPos.add(itJ.offset);
          } 
          else {//other joints:
                //apply trasnformation matrix from parent on offset
            RotRelativPos = applyMatPVect(itJ.parent.transMat, itJ.offset);
                //add transformed offset to parent position
            RotRelativPos.add(itJ.parent.position.get(itJ.parent.position.size()-1));
          }
          //store position
          itJ.position.add(RotRelativPos);
        }
      }
    }
  }
  
}

class Joint {

  String name;
  int isRoot = 0;
  int isEndSite = 0;
  Joint parent;
  PVector offset = new PVector();
  //transformation types (CHANNELS):
  String[] rotationChannels = new String[3];
  //current transformation matrix applied to this joint's children:
  float[][] transMat = {{1., 0., 0.},{0., 1., 0.},{0., 0., 1.}};

  //list of  , xyz position at each frame:
  ArrayList<PVector> position = new ArrayList<PVector>();
}


//-------------------------------------
// Functions --------------------------
//-------------------------------------


void drawGroundPlane( int size) {
  noStroke();
  fill(150,100);
  beginShape();
  vertex( -size, 0, -size ); 
  vertex( size, 0, -size ); 
  vertex( size, 0, size ); 
  vertex( -size, 0, size ); 
  endShape();
}

float[][] multMat(float[][] A, float[][] B) {//computes the matrix product AB
  int nA = A.length;
  int nB = B.length;
  int mB = B[0].length;
  float[][] AB = new float[nA][mB];
  for (int i=0;i<nA;i++) {
    for (int k=0;k<mB;k++) {
      if (A[i].length!=nB) { 
        println("multMat: matrices A and B have wrong dimensions! Exit.");
        exit();
      }
      AB[i][k] = 0.;
      for (int j=0;j<nB;j++) {
        if (B[j].length!=mB) { 
          println("multMat: matrices A and B have wrong dimensions! Exit.");
          exit();
        }
        AB[i][k] += A[i][j]*B[j][k];
      }
    }
  }
  return AB;
}

float[][] makeTransMat(float a, String channel) {
  //produces transformation matrix corresponding to channel, with argument a
  float[][] transMat = {{1., 0., 0.},{0., 1., 0.},{0., 0., 1.}};
  if (channel.equals("Xrotation")) {
    transMat[1][1] = cos(radians(a));
    transMat[1][2] = - sin(radians(a));
    transMat[2][1] = sin(radians(a));
    transMat[2][2] = cos(radians(a));
  }
  else if (channel.equals("Yrotation")) {
    transMat[0][0] = cos(radians(a)); 
    transMat[0][2] = sin(radians(a));
    transMat[2][0] = - sin(radians(a));
    transMat[2][2] = cos(radians(a));
  }
  else if (channel.equals("Zrotation")) {
    transMat[0][0] = cos(radians(a));
    transMat[0][1] = - sin(radians(a));
    transMat[1][0] = sin(radians(a));
    transMat[1][1] = cos(radians(a));
  }
  else {
    println("makeTransMat: unknown channel! Exit.");
    exit();
  }
  return transMat;
}

PVector applyMatPVect(float[][] A, PVector v) {
  //apply (square matrix) A to v (both must have dimension 3)
  for (int i=0;i<A.length;i++) {
    if (v.array().length!=3||A.length!=3||A[i].length!=3) {
      println("applyMatPVect: matrix and/or vector not of dimension 3! Exit.");
      exit();
    }
  }
  PVector Av = new PVector();
  Av.x = A[0][0]*v.x + A[0][1]*v.y + A[0][2]*v.z;
  Av.y = A[1][0]*v.x + A[1][1]*v.y + A[1][2]*v.z;
  Av.z = A[2][0]*v.x + A[2][1]*v.y + A[2][2]*v.z; 

  return Av;
}

float findFrameRate(MocapInstance mocapInstance) {
  
      if (mocap.frmRate!= mocap.frmRate) {
        println("findFrameRate: all mocaps don't have the same frame rate! Exit.");
        exit();
      }
    
    return mocap.frmRate;
}

float findFrameTime(MocapInstance mocapInstance) { 
    return mocap.frameTime;
}
