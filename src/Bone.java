import java.util.*;

import remixlab.bias.event.*;
import remixlab.proscene.*;
import remixlab.dandelion.geom.*;
import remixlab.dandelion.core.*;

//the angle of a bone is the angle btwn the bone and its parent

public class Bone extends GenericP5Frame{
  float radiusX = 5, radiusY = 5;
  int colour = -1;
  boolean selected = false;
  Skeleton skeleton;
  ArrayList<Bone> children = new ArrayList<Bone>();
  Bone parent = null;  
  Joint joint = null;
  Vec model_pos;
  float PI = (float) Math.PI;
  float prev_angle = 0;
  Vec prev_pos = new Vec(0,0,0);

  //Vars used for IKinematics purpose
  Vec final_ef_pos = new Vec(0,0,0);
  boolean is_end_effector = false;
  float weight = 1;
  float max_weight = 20;
  
  public void setupProfile(){
	  this.setClickBinding(MouseAgent.LEFT_ID, 1, "performClickEvent");
	  this.setClickBinding(MouseAgent.RIGHT_ID, 1, "performClickEvent");
	  this.setMotionBinding(MouseAgent.LEFT_ID, "performMotionEvent");
	  this.setMotionBinding(MouseAgent.RIGHT_ID, "performMotionEvent");
	  this.setMotionBinding(MouseAgent.WHEEL_ID, "performWheelEvent");	  
  }
  
  public Bone(Scene sc){
    super(sc);
    skeleton = new Skeleton(sc); 
    joint = new Joint();
    colour = sc.pApplet().color((float)Math.random()*255f, 
  		  (float) Math.random()*255f, (float) Math.random()*255f);
    setupProfile();
  }
  public Bone(Scene sc, Bone b, boolean isChild){
    super(sc);
    colour = sc.pApplet().color((float)Math.random()*255f, 
    		  (float) Math.random()*255f, (float) Math.random()*255f);
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
    setupProfile();
  }
  public Bone(Scene sc, Bone p, Bone c){
    super(sc);
    joint = new Joint();
    children.add(c); parent = p; joint = new Joint();
    p.children.add(this);
    setupProfile();
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
      Bone b = new Bone(scene(), this, false);
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
    float thresholdX = radiusX;
    float thresholdY = radiusY;
    Vec proj = scene().eye().projectedCoordinatesOf(position());
    if((Math.abs(x - proj.vec[0]) < thresholdY) && (Math.abs(y - proj.vec[1]) < thresholdY)){
      return true;      
    }
    return false;
  }

  
  public void performClickEvent(ClickEvent event) {
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
    	  Kinematics.last_selected_bone.selected = false;
      }
      Kinematics.last_selected_bone = this;
      //update Joint control
      Kinematics.control_frame.setupControlShape();
      selected = true;
    }
  }
  
  public void performMotionEvent(DOF2Event event) {
	if(event.id() == 39 || parent == null){  
		//if is end effector modify the final position
		if(is_end_effector){
		    Vec dif = screenToVec(Vec.multiply(new Vec(isEyeFrame() ? -event.dx() : event.dx(),
			        (scene().isRightHanded() ^ isEyeFrame()) ? -event.dy() : event.dy(), 0.0f), this.translationSensitivity()));    
			final_ef_pos.add(dif);
			return;
		}
		if(Kinematics.add_bone){
		  translate(screenToVec(Vec.multiply(new Vec(isEyeFrame() ? -event.dx() : event.dx(),
		      (scene().isRightHanded() ^ isEyeFrame()) ? -event.dy() : event.dy(), 0.0f), translationSensitivity())));    
		  skeleton.updateAngles();
		  return;
		}
		//Translate the skeleton Frame
		skeleton.frame.translate(skeleton.frame.screenToVec(Vec.multiply(new Vec(skeleton.frame.isEyeFrame() ? -event.dx() : event.dx(),
		    (scene().isRightHanded() ^ skeleton.frame.isEyeFrame()) ? -event.dy() : event.dy(), 0.0f), skeleton.frame.translationSensitivity())));    
	}
	if(event.id() == 37){ 
		rotate(event);
		if(!Kinematics.add_bone) Kinematics.control_frame.setupControlShape();
	}
  }

  public void performWheelEvent(DOF1Event event){
	  if(Kinematics.enable_mod_ef){
		  switchEndEffectorMode(event);
		  return;
	  }
	  //enable to change the radius of the bone
	  if(Kinematics.enable_mod_rad){
		  radiusX += event.dx()*0.125;
		  radiusX = radiusX <= 0 ? 0 : radiusX;
		  return;
	  }
	  //change the weight of the bone. More weight means it's harder to mov 
	  if(Kinematics.enable_mod_w){
		  weight += event.dx();
		  weight = weight > max_weight ? max_weight : weight;
		  weight = weight < 1 ? 1 : weight;
		  System.out.println("Bone Weight ----: " + weight);
	  }
  }
  
  public void switchEndEffectorMode(DOF1Event event){
	  is_end_effector = Kinematics.enable_ef;
	  if(is_end_effector){
		  final_ef_pos = position();
	  }
	  System.out.println("The bone is now end effector? : " + is_end_effector);
  }
  
  //Simulate rotation
  public void rotate(DOF2Event event){
	  Vec delta =  screenToVec(Vec.multiply(new Vec(isEyeFrame() ? -event.dx() : event.dx(),
	          (scene().isRightHanded() ^ isEyeFrame()) ? -event.dy() : event.dy(), 0.0f), translationSensitivity()));
	  float angle = getAngleFromPos(delta);
	  joint.angle = angle >= -1* PI && angle <= PI ? angle : joint.angle;	  	  
	  angleToPos();
	  skeleton.updateAngles();	  
  }
  
  //CHECK SCENE
  public void drawShape(){
      Vec aux = parent == null ? null : parent.inverseCoordinatesOf(new Vec(0,0,0));
      Vec aux2 = inverseCoordinatesOf(new Vec(0,0,0));
      Vec ortho = aux == null ? new Vec(1,0,0) : Vec.subtract(aux2, aux);
      ortho.normalize();
      float ang = (float) Math.atan2(ortho.x(), ortho.y());
      scene().pg().pushStyle();
      if(aux != null){
    	  //if the bone is lighter, it means more transparency
		  scene().pg().stroke(255,255,255, 50 + 200*weight/max_weight*1.f);        
		  scene().pg().line(aux2.x(),aux2.y(),aux2.z(),aux.x(), aux.y(), aux.z());
      }
      if(is_end_effector){
		  scene().pg().stroke(255,0,0,50);
		  Vec aux3 = final_ef_pos;
		  if(Vec.distance(aux2, aux3) != 0)
			  scene().pg().line(aux3.x(),aux3.y(),aux3.z(),aux2.x(), aux2.y(), aux2.z());
      }
      if(!selected)scene().pg().stroke(colour);
      else scene().pg().stroke(scene().pg().color(0,0,255));
      scene().pg().pushMatrix();
      scene().pg().translate(aux2.x(),aux2.y());
      scene().pg().rotate(-ang);
      scene().pg().line(-radiusX,0,radiusX,0);      
      scene().pg().popMatrix();      
      scene().pg().strokeWeight(radiusY*magnitude());
      scene().pg().point(aux2.x(),aux2.y());
      
      if(is_end_effector){
		  scene().pg().stroke(255,0,0,50);
    	  Vec aux3 = final_ef_pos;
		  if(Vec.distance(aux2, aux3) != 0)scene().pg().point(aux3.x(),aux3.y(),aux3.z());
      }
      scene().pg().popStyle();
  }
}
