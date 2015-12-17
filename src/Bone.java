import java.util.*;

import remixlab.bias.event.*;
import remixlab.proscene.*;
import remixlab.dandelion.geom.*;
import remixlab.dandelion.core.*;

//the angle of a bone is the angle btwn the bone and its parent

public class Bone extends InteractiveFrame{
  float radius = 10;
  int colour = -1;
  Skeleton skeleton;
  ArrayList<Bone> children = new ArrayList<Bone>();
  Bone parent = null;  
  Joint joint = null;
  Vec model_pos;
  float PI = (float) Math.PI;
  float prev_angle = 0;
  Vec prev_pos = new Vec(0,0,0);

  public Bone(Scene sc){
    super(sc);
    skeleton = new Skeleton(sc); 
    joint = new Joint();
    colour = sc.pApplet().color(0,255,0);
  }
  public Bone(Scene sc, Bone b, boolean isChild){
    super(sc);
    colour = sc.pApplet().color(0,255,0);    
    joint = new Joint();
    if(!isChild){
      parent = b;
      b.children.add(this);
    }
    else{
      children.add(b);
      b.parent = this;
    } 
    joint = new Joint();    
  }
  public Bone(Scene sc, Bone p, Bone c){
    super(sc);
    joint = new Joint();
    children.add(c); parent = p; joint = new Joint();
    p.children.add(this);
  }  

  void updateMainFrame(Frame frame){
    setReferenceFrame(frame);
  }
  
  public void createBone(float pos_x, float pos_y, Skeleton sk){
      System.out.println("entra");
      //Set initial orientatuion
      float angle = 0;
      //update();
      //add to an empty skeleton
      sk.bones.add(this);
      //relate to a new unconstrained Joint
      joint = new Joint((float)-1.*PI,PI, angle);
      //relate to parent frame
      updateMainFrame(skeleton.frame);
      this.translate(pos_x, pos_y);
      //this.rotate(new Quat(new Vec(0, 0, 1), angle));
      //relate the skeleton
      skeleton = sk;
      //keep a track of all the bones in the scene
      Kinematics.bones.add(this);
      System.out.println("sale");      
  }

  //Update the angle of the bone given a translation on its final point
  public void updateAngle(){
    Vec aux = parent == null ? new Vec(0,0,0) : parent.inverseCoordinatesOf(new Vec(0,0,0));
    Vec diff = Vec.subtract(inverseCoordinatesOf(new Vec(0,0,0)), aux);    
    float angle = (float)Math.atan2(diff.y(), diff.x());
    joint.angle = angle >= -1* PI && angle <= PI ? angle : joint.angle;
  }
  
  //Get the angle of a given position
  public float getAngleFromPos(Vec pos){
	  Vec aux = parent == null ? new Vec(0,0,0) : parent.inverseCoordinatesOf(new Vec(0,0,0));
	  Vec diff = Vec.subtract(inverseCoordinatesOf(pos), aux);    
      float angle = (float)Math.atan2(diff.y(), diff.x());	  
	  return angle;
  } 
  
  public void angleToPos(){
    Vec diff = translation();        
    float dim = diff.magnitude();
    this.setTranslation(dim*(float)Math.cos(joint.angle), dim*(float)Math.sin(joint.angle),0);
    System.out.println(joint.angle + " pos " + position());
  }


  //add a new bone to the skeleton 
  //modify the angle of the Joint of the related bone
  //TO DO - Add functionality to the change of hierarchy
  public void addBone(boolean asParent, Vec place){
      //create a bone as child
      float pos_x = place.x(); 
      float pos_y = place.y(); 
      float angle = (float)0.;
      Bone b = new Bone((Scene) scene, this, false);
      b.joint = new Joint((float)-1.* PI,PI, angle);
      //Apply transformations
      b.updateMainFrame(this);
      b.translate(pos_x, pos_y);
      //Relate the new bone with the corresponding lists
      b.skeleton = skeleton;
      skeleton.bones.add(b);
      Kinematics.bones.add(b);      
  }
  
  public Bone getRoot(){
	  if(this.parent == null) return this;
	  return this.parent.getRoot();
  }
  
  public ArrayList<Bone> getChildrenWS(){
	  ArrayList<Bone> bones = new ArrayList<Bone>();
	  ArrayList<Bone> queue = new ArrayList<Bone>();
	  queue.add(this);
	  while(!queue.isEmpty()){
		  Bone current = queue.remove(0);
		  for(Bone b : current.children){
			  queue.add(b);
			  bones.add(b);
		  }
	  }
	  return bones;
  }
  
  public boolean isAncester(Bone cur, Bone p){
    if(cur.parent == null) return false;
    if(cur.parent == p) return true;
    return isAncester(cur.parent, p);
  }

  public boolean isAncester(Bone p){
    return isAncester(this, p);
  }

  protected void spinExecution(){
    super.spinExecution();
    System.out.println("entra " + getCorrectAngle(rotation().angle()));
    System.out.println("entra g" + getCorrectAngle(orientation().angle()));
    joint.angle = getCorrectAngle(orientation().angle());
    skeleton.updateAngles();
  }  
  public void align() {
    super.align();
    System.out.println("entra " + getCorrectAngle(rotation().angle()));
    System.out.println("entra g" + getCorrectAngle(orientation().angle()));
    joint.angle = getCorrectAngle(orientation().angle());
    skeleton.updateAngles();
    //System.out.println("entra " +     rotation().normalize());
  }
  
  public float getCorrectAngle(float angle){
    float a = angle;
    //get Angle btwn -PI and PI
    while(a < -PI) a += 2*PI;
    while(a >  PI) a -= 2*PI;
    return a;
  }

  @Override
  public boolean checkIfGrabsInput(float x, float y){
    float threshold = radius;
    Vec proj = scene().eye().projectedCoordinatesOf(position());
    if((Math.abs(x - proj.vec[0]) < threshold) && (Math.abs(y - proj.vec[1]) < threshold)){
      return true;      
    }
    return false;
  }

  //scroll action will increase or decrease the detail of the shape
  @Override
  public void performCustomAction(DOF1Event event) {   
      gestureScale(event, wheelSensitivity());
  }

  
  @Override
  public void performCustomAction(ClickEvent event) {
    if(Kinematics.add_bone){
      if(event.id() == 39){
        Kinematics.removeSkeleton();
        return;
      }
      else{
        addBone(false, new Vec(50,0));
      }
    } 
    else{
      //change color and highlight as selected
      if(Kinematics.last_selected_bone != null){
    	  Kinematics.last_selected_bone.colour = ((Scene) scene).pApplet().color(0,255,0);
      }
      Kinematics.last_selected_bone = this;
      //update Joint control
      Kinematics.control_frame.setupControlShape();
      colour = ((Scene) scene).pApplet().color(0,0,255);
    }
  }
  
  @Override
  public void performCustomAction(DOF2Event event) {
	if(event.id() == 39 || parent == null){  
		if(Kinematics.add_bone){
		  translate(screenToVec(Vec.multiply(new Vec(isEyeFrame() ? -event.dx() : event.dx(),
		      (scene.isRightHanded() ^ isEyeFrame()) ? -event.dy() : event.dy(), 0.0f), translationSensitivity())));    
		  skeleton.updateAngles();
		  return;
		}
		//Translate the skeleton Frame
		skeleton.frame.translate(skeleton.frame.screenToVec(Vec.multiply(new Vec(skeleton.frame.isEyeFrame() ? -event.dx() : event.dx(),
		    (scene.isRightHanded() ^ skeleton.frame.isEyeFrame()) ? -event.dy() : event.dy(), 0.0f), skeleton.frame.translationSensitivity())));    
	}
	if(event.id() == 37){ 
		rotate(event);
		if(!Kinematics.add_bone) Kinematics.control_frame.setupControlShape();
	}
  }
  
  //Simulate rotation
  public void rotate(DOF2Event event){
	  Vec delta =  screenToVec(Vec.multiply(new Vec(isEyeFrame() ? -event.dx() : event.dx(),
	          (scene.isRightHanded() ^ isEyeFrame()) ? -event.dy() : event.dy(), 0.0f), translationSensitivity()));
	  float angle = getAngleFromPos(delta);
	  joint.angle = angle >= -1* PI && angle <= PI ? angle : joint.angle;	  	  
	  angleToPos();
	  skeleton.updateAngles();	  
  }
  
  //CHECK SCENE
  public void drawShape(){
      Vec aux = parent == null ? null : parent.inverseCoordinatesOf(new Vec(0,0,0));
      Vec aux2 = inverseCoordinatesOf(new Vec(0,0,0));
      ((Scene) scene).pg().pushStyle();
      if(aux != null){
		  ((Scene) scene).pg().stroke(255,255,255);        
		  ((Scene) scene).pg().line(aux2.x(),aux2.y(),aux2.z(),aux.x(), aux.y(), aux.z());
      }
      ((Scene) scene).pg().strokeWeight(radius);
      ((Scene) scene).pg().stroke(colour);
      ((Scene) scene).pg().point(aux2.x(),aux2.y(),aux2.z());
      ((Scene) scene).pg().popStyle();
  }
}
