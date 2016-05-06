import java.util.*;
import java.util.Map.Entry;

import processing.core.*;
import remixlab.dandelion.geom.Vec;

public class Skinning {
	public ArrayList<Vertex> vertices;
	
	public class Vertex{
		PVector base;
		PVector v; 
		int idx_shape;
		PShape shape;
		ArrayList<VertexAttribs> attribs;
		
		
		public class VertexAttribs{
			Bone bone;
			double weight;
			float initial_angle;
			PVector initial_position;
			
			public VertexAttribs(Bone b, double w, float ini_ang, PVector ini_pos){
				bone = b;
				weight= w;
				initial_angle = ini_ang;
				initial_position = ini_pos;
			}			
		}		
				
		public Vertex(PVector b, int i){
			base = b.get();
			v = base.get();
			idx_shape = i;
			attribs = new ArrayList<VertexAttribs>();
		}		
		
		public void applyTransformation(Utilities.CustomFrame model){	    
			PVector final_pos = new PVector(0,0);
			for(Vertex.VertexAttribs ats : this.attribs){
		    	this.v = model.getShape().getVertex(idx_shape);
			    Bone bone = ats.bone;
			    float rot_angle = bone.joint.angle - ats.initial_angle;
	    	    Vec mov = Vec.subtract(bone.parent.model_pos, new Vec(ats.initial_position.x, ats.initial_position.y));
	            //do all transformations in model space
			    Vec vec = new Vec(v.x,v.y,v.z);
			    Vec rot = Vec.subtract(vec, new Vec(ats.initial_position.x, ats.initial_position.y));
			    rot.rotate(rot_angle);
			    Vec new_pos = Vec.add(rot,new Vec(ats.initial_position.x, ats.initial_position.y));        
			    //apply translation	        
			    new_pos.add(mov);
			    //apply the weights
			    PVector new_pos_w = new PVector(new_pos.x(), new_pos.y());
			    new_pos_w.mult((float)ats.weight);
			    final_pos.add(new_pos_w);
			    ats.initial_position = new PVector(bone.parent.model_pos.x() , bone.parent.model_pos.y());
			    ats.initial_angle = bone.joint.angle;	 
				System.out.println("cambiaaa: " + v + "\n" + final_pos);
			}
			if(PVector.dist(final_pos, v) > 0 ){
				System.out.println("cambiaaa: " + v + "\n" + final_pos);
			} 
			this.v = final_pos;
		}
		
		public void addAttrib(Bone b, float dist, Vec ini_pos ){
			attribs.add(new Vertex.VertexAttribs(b,
					dist, b.joint.angle, new PVector (ini_pos.x(), ini_pos.y())));
		}
		
	}
	
	public void setup(Utilities.CustomFrame model, ArrayList<Bone> bones){
		PShape shape = model.getShape();
		vertices = new ArrayList<Vertex>();
		for(int i = 0; i < shape.getVertexCount(); i++){
			PVector v = shape.getVertex(i);
			Vertex vertex = new Vertex(v, i);
			Vec vec = new Vec(v.x,v.y);
			double total_dist = 0.;
			for(Bone b : bones){
		  		if(b.parent == null) continue;
				double dist = Utilities.getDistance(vec, b)[1];
				dist = 1./Math.exp(dist);
				total_dist += dist;
				Vec ini_pos = model.coordinatesOf(b.parent.position().get());				
				vertex.addAttrib(b,	(float) dist, ini_pos);
			}
			//The more near, the more weight the bone applies
			for(Vertex.VertexAttribs ats : vertex.attribs){
				ats.weight = ats.weight/(total_dist*1.); 
			}
			vertices.add(vertex);
		}
	}
	
}
