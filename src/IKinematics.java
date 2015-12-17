import java.util.*;

import processing.core.*;
import remixlab.dandelion.geom.*;
import remixlab.proscene.Scene;


public class IKinematics {
	/*Contains methods to apply IK
	more info look at 
	http://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
	http://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/SdlsPaper.pdf
	*/
	static LaplacianDeformation laplacian;
	static boolean debug = false;
	//public  class InverseKinematics{
	  //s : end effector position
	  //theta: rotation joint (1DOF) for each axis
	  //s(theta): end effectors are functions of theta
	  //v: Vector pointing to the axis of rot
	  //P: position of the join
	  //D(S)/D(theta) =  Vj X (Si - Pj)
	  //Vj for 2d is (0,0,1), for 3D could take 3 values
	  public Vec calculateDiff(Vec Pj, Vec Si, Vec Vj){
	    Vec sub = new Vec(0,0,0); 
	    Vec.subtract(Si, Pj, sub);
	    Vec diff = new Vec(0,0,0);
	    Vec.cross(Vj, sub, diff);
	    return diff;
	  }
	  
	  //Calculate 
	  public float[][] calculateJacobian(ArrayList<Bone> joints, ArrayList<Bone> end_effectors){
	    float[][] jacobian = new float[end_effectors.size()*3][joints.size() -1];
	    //calc 3 rows corrsponding to a end effector    
	    int i = 0;
	    for(Bone s_i : end_effectors){
	      int j = 0;
	      for(Bone theta_j : joints){
	        //check if the joint could move the end effector
	        if(!s_i.isAncester(theta_j) || theta_j.parent == null){
	          jacobian[i][j] = 0;
	          jacobian[i+1][j] = 0;
	          jacobian[i+2][j] = 0;         
	        }else{
	          Vec Pj = theta_j.parent.position();
	          Pj = new Vec(Pj.x(), Pj.y(), (float)0.0);  
	          Vec Si = s_i.position();
	          Si = new Vec(Si.x(), Si.y(), (float)0.0);  
	          Vec res = calculateDiff(Pj, Si, new Vec(0,0,1));        
	          jacobian[i][j] = res.x();
	          jacobian[i+1][j] = res.y();
	          jacobian[i+2][j] = res.z();
	        }          
	        j++;
	      }
	      i+=3;
	    }  
	    return jacobian;
	  } 
	  
	  public float[] calculateError(ArrayList<Bone> s, ArrayList<Vec> t){
	    float[] e = new float[3*s.size()];
	    for(int i = 0; i < s.size(); ){
	      Vec ti = t.get(i);
	      Vec si = s.get(i).position();
	      e[i++] = ti.x() - si.x();
	      e[i++] = ti.y() - si.y();
	      e[i++] = ti.z() - si.z();
	    }
	    return e;
	  }
	  
	  public static float[] getDeltaTheta(float alpha, float[] e){
	    float[] deltaTheta = new float[e.length];
	    //
	    return deltaTheta;
	  }  
	  public static float getDistance(Vec vv, Bone b){
	    if(b.parent == null) return 99999;
	    //is the distance btwn line formed by b and its parent and v
	    Vec line = Vec.subtract(b.position(), b.parent.position());
	    Vec va = Vec.subtract(vv, b.parent.position());
	    float dot = Vec.dot(va, line);
	    float mag = line.magnitude();
	    float u  = dot*(float)1./(mag*mag);
	    Vec aux = new Vec();
	    if(u >= 0 && u <=1){
	      aux = new Vec(b.parent.position().x() + u*line.x(), b.parent.position().y() + u*line.y()); 
	      aux = Vec.subtract(aux, vv);
	    }
	    if(u < 0){
	      aux = Vec.subtract(b.parent.position(), vv); 
	    }
	    if(u > 1){
	      aux = Vec.subtract(b.position(), vv); 
	    }
	    return aux.magnitude();
	  }

	  //Skinning algorithm
	  static int counter = -1;
	  public static void applyTransformations(Utilities.CustomModelFrame model){
	    if(laplacian == null) return;
	    counter++;
	    if(counter == 1){
	    	counter = 0;
	    	//if(debug) return;
	    }
	    
	    for(Skeleton s : Kinematics.skeletons){
	    	Bone root = s.frame.getRoot();
	    	for(Bone bone : root.getChildrenWS()){
	    		//update old values
		    	float rot_angle = bone.joint.angle - bone.prev_angle;      
		    	Vec mov = Vec.subtract(bone.parent.model_pos, bone.prev_pos);
		        Vec rot = Vec.subtract(bone.model_pos, bone.parent.model_pos);
				rot.rotate(rot_angle); rot.add(bone.parent.model_pos);
				//apply translation
				rot.add(mov);
				bone.model_pos = rot;
				bone.prev_pos = bone.parent.model_pos.get();
				bone.prev_angle = bone.joint.angle;
	    	}
	    }
	    	    
	    for(LaplacianDeformation.Anchor anchor : laplacian.anchors){	    	
	    	PVector final_pos = new PVector(0,0);
	    	anchor.pos = model.shape.getVertex(anchor.vertex.idx_shape);
	        if(debug)System.out.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& size ats :" + anchor.attribs.size());	    	
	    	for(LaplacianDeformation.Anchor.AnchorAttribs ats : anchor.attribs){
		    	//get the movement of the bone
		    	Bone bone = ats.related_bone;
		    	float rot_angle = bone.joint.angle - ats.initial_angle;      
		    	Vec mov = Vec.subtract(bone.parent.model_pos, new Vec(ats.initial_pos.x, ats.initial_pos.y));
		        //do all transformations in model space
		        Vec vec = new Vec(anchor.pos.x,anchor.pos.y,anchor.pos.z);
		        Vec rot = Vec.subtract(vec, bone.parent.model_pos);
		        rot.rotate(rot_angle);
		        Vec new_pos = Vec.add(rot,bone.parent.model_pos);        
		        //apply translation	        
		        new_pos.add(mov);
		        if(debug)System.out.println("rot: "  + rot);
		        if(debug)System.out.println("mov: "  + mov);
		        if(debug)System.out.println("pos : " + vec + " new pos : " + new_pos);	        
		        //apply the weights
		        PVector new_pos_wi = anchor.pos.get();
		        new_pos_wi.mult(ats.weight);
		        final_pos.add(new_pos_wi);
		        if(debug)System.out.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& asdfasd sad _  :" + ats.weight);
		        PVector new_pos_w = new PVector(new_pos.x(), new_pos.y());
		        new_pos_w.mult(1 - ats.weight);
		        final_pos.add(new_pos_w);
		        if(rot_angle > 0.01){
		        	anchor.weight = 1f;
		        }else{
		        	anchor.weight = 0.01f;
		        }
		        //anchor.pos = new_pos_w;
		        //update old values
				ats.initial_pos = new PVector(bone.parent.model_pos.x() , bone.parent.model_pos.y());
				//rot = Vec.subtract(bone.model_pos, bone.parent.model_pos);
				//rot.rotate(rot_angle); rot.add(bone.parent.model_pos);
				//apply translation
				//rot.add(mov);
				//bone.model_pos = rot;
				ats.initial_angle = bone.joint.angle;	 
				if(debug)System.out.println("final_pos " + final_pos);
	    	}
	    	anchor.pos = final_pos;
	    	anchor.pos.mult(1.f/anchor.attribs.size());
	    }
	    //solve the laplacian system
		ArrayList<PVector> coords = laplacian.solveLaplacian();
	    for(LaplacianDeformation.Vertex v : laplacian.vertices){
	    	model.shape.setVertex(v.idx_shape, coords.get(v.idx));
	    }
	  }
	  
	  //Skinning based on laplacian deformation
	  public static void execSkinning(Utilities.CustomModelFrame model, ArrayList<Bone> bones){
	    laplacian = new LaplacianDeformation();
	    //update laplacian info of each bone
	  	laplacian.setup(model.shape());
	  	//for each bone get its position and  add an Anchor to nearest vertices
	  	for(int i = 0; i < bones.size(); i++){
	  		bones.get(i).model_pos = model.coordinatesOf(bones.get(i).position().get());
	  		bones.get(i).prev_angle = bones.get(i).joint.angle;
	  		if(bones.get(i).parent == null) continue;
	  		bones.get(i).prev_pos = model.coordinatesOf(bones.get(i).parent.position().get());
	  		laplacian.addAnchorByDist(laplacian.anchors, model, bones.get(i), i, 0.05f);
	  	}
	  	laplacian.calculateLaplacian();
	  }  
	//}
}
