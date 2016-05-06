import processing.core.*;
/*First approach in Inverse Kinematic*/

import java.util.*;

//-----------------------------------------------
//Proscene
//Use InteractiveModelFrame and override actions
import remixlab.proscene.*;
import remixlab.dandelion.geom.*;

/*
Sebastian Chaparro
July 2 2015
*/

public class Kinematics extends PApplet{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	//Processing variables
	//PGRAPHICS
	PGraphics main_graphics;
	PGraphics aux_graphics;
	PGraphics control_graphics;

	//SCENES------------------------------------------
	/*Basically 2 scenes are required: one to draw the main Object,
	the other one to control the world as an sphere */
	static Scene main_scene;
	static Scene aux_scene;
	static int TOKEN_W = 5;
	static int TOKEN_H = 10;
	static int TOKEN_DIM = 1;
	static Bone last_selected_bone = null;
	JointControl join_control = null;
	static JointControl control_frame;
	static final int all_width = 600;
	static final int all_height = 620;
	boolean showAid = true;
	final int aux_pos_x = all_width-all_width/4;
	final int aux_pos_y = all_height-all_height/3;
	//Some models
	static Utilities.CustomFrame original_fig;
	

	//This is gonna be a serie of tokens to manipulate
	static ArrayList<Skeleton> skeletons = new ArrayList<Skeleton>();
	static ArrayList<Bone> bones = new ArrayList<Bone>();
	
	static boolean laplacian = true;
	
	public static void main(String[] args){
		PApplet.main(Kinematics.class.getName());		
	}
	
	public void settings(){
	  size(600, 620, P2D);		
	}

	public void setup(){
	  main_graphics = createGraphics(all_width,all_height,P2D);
	  main_scene = new Scene(this, main_graphics);
	  aux_graphics = createGraphics(all_width/4,all_height/3,P2D);
	  aux_scene = new Scene(this, aux_graphics, aux_pos_x, aux_pos_y);    
	  main_scene.setAxesVisualHint(false);
	  main_scene.setGridVisualHint(false);
	  aux_scene.setAxesVisualHint(true);
	  aux_scene.setGridVisualHint(true);
	  main_scene.setRadius(50);
	  
	  //main_scene.mouseAgent().setButtonBinding(Target.FRAME, RIGHT, DOF2Action.CUSTOM);
	  //main_scene.mouseAgent().setButtonBinding(Target.FRAME, LEFT, DOF2Action.CUSTOM);
	  //main_scene.mouseAgent().setClickBinding(Target.FRAME, LEFT, ClickAction.CUSTOM);
	  //main_scene.mouseAgent().setClickBinding(Target.FRAME, RIGHT, ClickAction.CUSTOM);
	  //main_scene.mouseAgent().setWheelBinding(Target.FRAME, DOF1Action.CUSTOM);  
	  //aux_scene.mouseAgent().setButtonBinding(Target.FRAME, RIGHT, DOF2Action.CUSTOM);
	  //aux_scene.mouseAgent().setButtonBinding(Target.FRAME,  LEFT , DOF2Action.CUSTOM);
	  
	  control_frame = new JointControl(aux_scene);
	  ImageProcessing ip = new ImageProcessing(this);
	  ip.setupFigure();
	}

	public void draw(){
	  IKinematics.executeDLS();		
	  handleAgents();  
	  main_graphics.beginDraw();
	  main_scene.beginDraw();
	  main_graphics.background(0);
	  original_fig.draw();
	  drawBones();
	  IKinematics.drawAnchors(main_scene, original_fig);
	  main_scene.endDraw();
	  main_graphics.endDraw();    
	 image(main_graphics, main_scene.originCorner().x(), main_scene.originCorner().y());
	  if (showAid) {
	    aux_graphics.beginDraw();
	    aux_scene.beginDraw();
	    aux_graphics.background(125, 125, 125, 125);
	    aux_scene.drawFrames();
	    aux_scene.endDraw();
	    aux_graphics.endDraw();    
	    image(aux_graphics, aux_scene.originCorner().x(), aux_scene.originCorner().y());
	  }
	  main_graphics.beginDraw();
	  main_scene.beginDraw();
	  //IKinematics.applyTransformations(original_fig);
	  main_scene.endDraw();
	  main_graphics.endDraw();    
	}

	int drag_mode = -1;
	void handleAgents() {
	  aux_scene.disableMotionAgent();
	  aux_scene.disableKeyboardAgent();
	  main_scene.disableMotionAgent();
	  main_scene.disableKeyboardAgent();
	  if ((mouseX >= all_width - aux_scene.width()) && mouseY >= all_height - aux_scene.height()) {
	    aux_scene.enableMotionAgent();
	    aux_scene.enableKeyboardAgent();
	  }else if(drag_mode == -1) {
	    main_scene.enableMotionAgent();
	    main_scene.enableKeyboardAgent();
	  }
	}	

	//HANDLE SOME MOUSE AND KEYBOARD ACTIONS
	static boolean add_bone = false;
	//Bone.add_bone = false;
	public void mousePressed(){
	  /*for(Bone f : bones){
	    if(f.grabsInput(main_scene.motionAgent())){
	      last_selected_bone = f;          
	      return;
	    }
	  }*/
	  if(add_bone){
	    if ((mouseX >= all_width - aux_scene.width()) && mouseY >= all_height - aux_scene.height()) return;
	    if(mouseButton == LEFT){
	      for(Bone f : bones){
	        if(f.checkIfGrabsInput(mouseX,mouseY)){
	          return;
	        }
	      }
	      Vec point_world = main_scene.eye().unprojectedCoordinatesOf(new Vec(mouseX, mouseY));
	      skeletons.add(new Skeleton(main_scene,point_world.x(), point_world.y()));
	    }
	    if(mouseButton == RIGHT){
	      //removeSkeleton();
	    }
	  }
	}
	boolean temp = false;
	static boolean enable_ef = false;
	static boolean enable_mod_ef = false;
	static boolean enable_mod_rad = true;
	static boolean enable_mod_w = false;

	public void keyPressed(){  
	  //if(key == 'x' || key== 'X'){
	  //  temp = !temp;
	  //  if(temp) main_scene.removeModel(original_fig);
	  //  else main_scene.addModel(original_fig);
	  //}
	  
	  if(key=='b' || key=='B'){
	    add_bone = !add_bone;
	    if(last_selected_bone != null){
	      last_selected_bone.selected = false;
	    }
	    last_selected_bone = null;
	  }
	  
	  if(key == 'z' || key == 'Z'){
	    if(last_selected_bone != null){
	    	ArrayList<Bone> bones = last_selected_bone.skeleton.frame.getChildrenWS();
	    	bones.add(0, last_selected_bone.skeleton.frame);
	    	//IKinematics.execSkinning(original_fig,bones);
	    	IKinematics.execSkinningLinearBlending(original_fig,bones);
	    	System.out.println("sale");
	    }
	  }
	  
	  if(key == 'l' || key == 'L'){
		  //IKinematics.applyTransformations(original_fig);
		  IKinematics.applyTransformationsLinearBlending(original_fig);		  
	  }

	  if(key == 'n' || key == 'N'){
		  enable_ef = !enable_ef;
	  }
	  if(key == 'm' || key == 'M'){
		  IKinematics.executeDLS();
	  }
	  if(key == '1'){
		  enable_mod_ef = true;
		  enable_mod_rad = false;
		  enable_mod_w = false;		  
	  }
	  if(key == '2'){
		  enable_mod_ef = false;
		  enable_mod_rad = true;
		  enable_mod_w = false;		  
	  }
	  if(key == '3'){
		  enable_mod_ef = false;
		  enable_mod_rad = false;
		  enable_mod_w = true;		  
	  }
	}

	//change by a deph search
	void drawBones(){
	  for(Skeleton s : skeletons){
	    drawBones(s.frame);
	  }
	}

	void drawBones(Bone root){
	    main_scene.pg().pushMatrix();
	    //root.applyWorldTransformation();
	    main_scene.applyWorldTransformation(root);
	    //main_scene.drawAxes(40);    
	    main_scene.pg().popMatrix();
	    root.drawShape();    
	    for(Bone child : root.children) drawBones(child);
	}

	//Token mods:
	public static void removeSkeleton(){
	  Skeleton sk = null;
	  for (int i = 0; i < bones.size(); i++){ 
	    if (bones.get(i).grabsInput(main_scene.motionAgent())){
	      sk = bones.get(i).skeleton;
	      break;
	    }
	  }
	  if(sk == null) return;
	  for(Bone b : sk.bones){
	    //main_scene.removeModel(b);
	    bones.remove(b);
	  }
	  skeletons.remove(sk);  
	  last_selected_bone = null;
	}	
}







