import java.util.*;

import org.la4j.LinearAlgebra.InverterFactory;
import org.la4j.Matrix;
import org.la4j.inversion.MatrixInverter;
import org.la4j.matrix.dense.Basic1DMatrix;
import org.la4j.matrix.dense.Basic2DMatrix;
import org.la4j.vector.dense.BasicVector;

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
	static float d_max = -1;
	static float lambda = 0.1f;
	//public  class InverseKinematics{
	  //s : end effector position
	  //theta: rotation joint (1DOF) for each axis
	  //s(theta): end effectors are functions of theta
	  //v: Vector pointing to the axis of rot
	  //P: position of the join
	  //D(S)/D(theta) =  Vj X (Si - Pj)
	  //Vj for 2d is (0,0,1), for 3D could take 3 values
	  public static Vec calculateDiff(Vec Pj, Vec Si, Vec Vj){
	    Vec sub = new Vec(0,0,0); 
	    Vec.subtract(Si, Pj, sub);
	    Vec diff = new Vec(0,0,0);
	    Vec.cross(Vj, sub, diff);
	    return diff;
	  }
	  
	  //Execute Damped Least Squares Algorithm
	  //Jacobian is calculated each time this method is called, assuming that it is not to costly
	  public static void executeDLS(){
		  //get all end effectors
	    for(Skeleton s : Kinematics.skeletons){
	    	ArrayList<Bone> end_effectors = new ArrayList<Bone>();
	    	Bone root = s.frame.getRoot();
	    	for(Bone bone : root.getChildrenWS()){
	    		//add all end effectors
	    		if(bone.is_end_effector)  end_effectors.add(bone);
	    	}
	    	if(end_effectors.size() == 0) continue;
	    	//Get the jacobian
	    	double[][] jacobian = calculateJacobian(s.bones, end_effectors);
	    	//Get Delta theta
	    	Basic2DMatrix J = new Basic2DMatrix(jacobian);
	    	System.out.println("JACO : \n\n\n\n" + J.toString());
	    	Basic2DMatrix J_T = (Basic2DMatrix) J.transpose();	 
	    	
	    	//use a weigthed approach
	    	double[][] W_mat = new double[s.bones.size()][s.bones.size()];
	    	for(int i = 0; i < W_mat.length; i++){
	    		W_mat[i][i] = 1./(s.bones.get(i).weight);
	    	}
	    	//Basic2DMatrix W_i = new Basic2DMatrix(W_mat);
	    	//J_T = (Basic2DMatrix) W_i.multiply(J_T);
	    	Basic2DMatrix JJ_T = (Basic2DMatrix) J.multiply(J_T);
	    	//construct identity
	    	Basic2DMatrix lambda_2_diag = Basic2DMatrix.identity(J.rows());
	    	lambda_2_diag = (Basic2DMatrix) lambda_2_diag.multiply(lambda*lambda);
	    	Matrix term = JJ_T.add(lambda_2_diag);
	    	term = term.withInverter(InverterFactory.GAUSS_JORDAN).inverse();
	    	term = J_T.multiply(term);
	    	System.out.println("TERM : " + term.toString());
	    	Vec[] e_vec_t, e_vec;
	    	double e_t = 999, prev_e_t = 999;	    	
	    	int it = 0;
	    	//clamp each e_i
	    	setDMax(s.bones);
	    	//Get error (e = t - s)
	    	e_vec_t = calculateError(end_effectors);
	    	e_vec = new Vec[e_vec_t.length];
	    	for(int i = 0; i < e_vec.length; i++){
	    		e_vec[i] = clampMag(e_vec_t[i], d_max);
	    	}
	    	//change the way that e is stored
	    	double[] e = new double[e_vec.length*2];
	    	for(int i = 0; i < e_vec.length*2;){
	    		e[i] = e_vec[i/2].x();i++;
	    		e[i] = e_vec[i/2].y();i++;
	    	}
	    	BasicVector e_v = new BasicVector(e);
	    	System.out.println("error _v -----------------: + \n" + e_v.toString());
	    	
	    	//Apply Weights
	    	double[][] w = new double[s.bones.size()][s.bones.size()];
	    	double w_t = 0;
	    	for(int i = 0; i < w.length; i++){
	    		w[i][i] = 1./s.bones.get(i).weight;
	    		w_t += w[i][i];		
	    	}
	    	for(int i = 0; i < w.length; i++)w[i][i] = w[i][i]*1./w_t;
	    	Basic2DMatrix w_vec = new Basic2DMatrix(w);
	    	BasicVector delta_theta = (BasicVector) term.multiply(e_v);
	    	delta_theta = (BasicVector) w_vec.multiply(delta_theta);
	    	System.out.println("delta theta -----------------: + \n" + delta_theta.toString());
	    	//sum delta 
	    	applyDeltaTetha(s.bones, delta_theta.toArray());
	    	e_t = 0;
	    	for(int i = 0; i < e_vec_t.length; i++){
	    		e_t += e_vec_t[i].magnitude()*e_vec_t[i].magnitude();
	    	}
	    	e_t = Math.sqrt(e_t);
	    	System.out.println("Error mag : " + e_t);
	    }
	  }
	  
	  //sum delta
	  public static void applyDeltaTetha(ArrayList<Bone> joints, double[] delta){
		  int i = 0;
		  System.out.println("delta size : " + delta.length);
		  System.out.println("num joints : " + joints.size());
		  for(Bone theta : joints){
			  float delta_angle = (float)delta[i];	  
			  if(theta.parent == null){
				  System.out.println("cambio: " + delta_angle);
				  i++;
				  continue;
			  } 
  		      while(delta_angle < -Math.PI) delta_angle += 2*(float)Math.PI;
			  while(delta_angle >  Math.PI) delta_angle -= 2*(float)Math.PI;
			  delta_angle = Math.abs(delta_angle) > Math.PI/4. ? delta_angle*0.1f : delta_angle;
			  float new_angle = theta.joint.angle + delta_angle;
			  new_angle = new_angle >= Math.PI ? new_angle - (float)Math.PI*2 : new_angle;
			  new_angle = new_angle <= -Math.PI ? new_angle + (float)Math.PI*2 : new_angle;			  
			  new_angle = new_angle > theta.joint.max_angle ? theta.joint.max_angle : new_angle;
			  new_angle = new_angle < theta.joint.min_angle ? theta.joint.min_angle : new_angle;
			  theta.joint.angle = new_angle;
			  theta.angleToPos();
			  i++;
		  }
	  }
	  
	  //Calculate 
	  public static double[][] calculateJacobian(ArrayList<Bone> joints, ArrayList<Bone> end_effectors){
	    double[][] jacobian = new double[end_effectors.size()*2][joints.size()];
	    //calc 3 rows corresponding to a end effector    
	    int i = 0;
	    for(Bone s_i : end_effectors){
	      //clamp target dist to a reachable pos
	      float max_dist = 0;
	      Bone aux =  s_i;
	      while(aux.parent != null){
	    	  max_dist += Vec.distance(aux.position(), aux.parent.position());
	    	  aux = aux.parent;
	      }
	      Bone r = s_i.getRoot();
	      Vec new_pos = Vec.subtract(s_i.final_ef_pos, r.position());
	      float dist = new_pos.magnitude();
	      if(dist > max_dist){
	    	  new_pos = Vec.subtract(s_i.final_ef_pos, r.position());
		      new_pos.normalize();
		      new_pos.multiply(max_dist);
		      new_pos.add(r.position());
	    	  s_i.final_ef_pos = new_pos;
	      }
	    	
	      int j = 0;
	      for(Bone theta_j : joints){
	        //check if the joint could move the end effector
	        if((!s_i.isAncester(theta_j) || theta_j.parent == null) && theta_j != s_i){
	          jacobian[i][j] = 0;
	          jacobian[i+1][j] = 0;
	        }else{
	          Vec Pj = theta_j.parent.position();
	          Pj = new Vec(Pj.x(), Pj.y(), (float)0.0);  
	          Vec Si = s_i.position();
	          Si = new Vec(Si.x(), Si.y(), (float)0.0);  
	          Vec res = calculateDiff(Pj, Si, new Vec(0,0,1));        
	          jacobian[i][j] = res.x();
	          jacobian[i+1][j] = res.y();
	        }          
	        j++;
	      }
	      i+=2;
	    }  
	    return jacobian;
	  } 
	  
	  public static Vec[] calculateError(ArrayList<Bone> s){
	    Vec[] e = new Vec[s.size()];
	    for(int i = 0; i < s.size(); ){
	      Vec ti = s.get(i).final_ef_pos;
	      Vec si = s.get(i).position();
	      e[i++] = Vec.subtract(ti, si);
	    }
	    return e;
	  }
	  
	  //Setting D_max as the same constant for all the end effectors
	  public static void setDMax(ArrayList<Bone> bones){
		  float avg_mag = 0;
		  for(Bone b : bones){
			  if(b.parent != null){
				  Vec dist = Vec.subtract(b.parent.position(), b.position());
				  avg_mag += dist.magnitude();
			  }
		  }
		  d_max = avg_mag/(bones.size()*5.f); 
	  }
	  
	  public static Vec clampMag(Vec w, float d){
		  if(w.magnitude() <= d) return w;
		  Vec v = w.get();
		  v.normalize();
		  v.multiply(d);
		  return v;
	  }
	  

	  //Skinning algorithm
	  static int counter = -1;
	  public static void applyTransformations(Utilities.CustomFrame model){
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
		        Vec rot = Vec.subtract(bone.model_pos, bone.prev_pos);
				rot.rotate(rot_angle); rot.add(bone.prev_pos);
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
		        Vec rot = Vec.subtract(vec, new Vec(ats.initial_pos.x, ats.initial_pos.y));
		        rot.rotate(rot_angle);
		        Vec new_pos = Vec.add(rot,new Vec(ats.initial_pos.x, ats.initial_pos.y));        
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
	        	anchor.weight = 1f;
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
	    laplacian.getLHS();
		ArrayList<PVector> coords = laplacian.solveLaplacian();
	    for(LaplacianDeformation.Vertex v : laplacian.vertices){
	    	model.shape.setVertex(v.idx_shape, coords.get(v.idx));
	    }
	  }
	  
	  //Skinning based on laplacian deformation
	  public static void execSkinning(Utilities.CustomFrame model, ArrayList<Bone> bones){
	    laplacian = new LaplacianDeformation();
	    //update laplacian info of each bone
	  	laplacian.setup(model.shape());
	  	//for each bone get its position and  add an Anchor to nearest vertices
	  	for(int i = 0; i < bones.size(); i++){
	  		bones.get(i).model_pos = model.coordinatesOf(bones.get(i).position().get());
	  		bones.get(i).prev_angle = bones.get(i).joint.angle;
	  		if(bones.get(i).parent == null) continue;
	  		bones.get(i).prev_pos = model.coordinatesOf(bones.get(i).parent.position().get());
	  		laplacian.addAnchorByDist(laplacian.anchors, model, bones.get(i), i, 0.6f);
	  	}
	  	laplacian.calculateLaplacian();
	  }  
	//}
	  
	  public static void drawAnchors(Scene sc, Utilities.CustomFrame model){
		  if(laplacian == null) return;
		  for(LaplacianDeformation.Anchor anchor : laplacian.anchors){
		      PVector v = model.shape.getVertex(anchor.vertex.idx_shape);
		      Vec v_w = model.inverseCoordinatesOf(new Vec(v.x, v.y));
			  for(LaplacianDeformation.Anchor.AnchorAttribs ats : anchor.attribs){
				  sc.pg().pushStyle();
				  sc.pg().strokeWeight(1);
				  sc.pg().stroke(ats.related_bone.colour);
				  sc.pg().point(v_w.x(), v_w.y());
				  sc.pg().popStyle();
			  }
		  }
	  }
}
