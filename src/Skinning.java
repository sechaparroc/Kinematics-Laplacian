import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import processing.core.PVector;
import remixlab.dandelion.geom.Vec;

public class Skinning {
	
	public static ArrayList<Vertex> vertices = new ArrayList<Vertex>();
	
	public static class Attribs{
		Bone bone;
		float weight;
		float initial_angle;
		Vec initial_pos;
		
		public Attribs(Bone b, float w, float a, Vec v){
			bone = b;
			weight = w;
			initial_angle = a;
			initial_pos = v.get();
		}
	}

	public static class Vertex{
				
		int idx;
		Utilities.CustomFrame model;
		ArrayList<Attribs> related_bones; 	
		float total_dist = 0;
		
		public Vertex(int i, Utilities.CustomFrame model){
			idx = i;
			this.model = model;
			related_bones = new ArrayList<Attribs>();
			total_dist = 0;
		}
		
		public PVector v(){
			return model.shape.getVertex(idx);
		}
		
		public void set(float x, float y, float z){
			model.shape.setVertex(idx, x, y, z);
		}
		
		public void addBone(Bone b, float dist){
			if(b.parent == null || related_bones.contains(b)) return;
			Attribs attrs = new Attribs(b, dist, b.joint.angle, model.coordinatesOf(b.parent.position().get()));
			related_bones.add(attrs);			
			total_dist += dist;			
		}
		
		public void normalizeWeights(){
			for(Attribs attrs : related_bones){
				attrs.weight /= total_dist;  
			}
		}
		
		public void applyTransformation(){
			Vec final_pos = new Vec(0,0);
			for(Attribs ats  : related_bones){
		    	//get the movement of the bone
		    	Bone bone = ats.bone;
		    	float rot_angle = bone.joint.angle - ats.initial_angle;      
		    	Vec mov = Vec.subtract(bone.parent.model_pos, ats.initial_pos);
		        //do all transformations in model space
		        Vec vec = new Vec(v().x,v().y);
		        Vec rot = Vec.subtract(vec, ats.initial_pos);
		        rot.rotate(rot_angle);
		        Vec new_pos = Vec.add(rot,ats.initial_pos);        
		        //apply translation
		        new_pos.add(mov);
		        //apply the weights
		        new_pos.multiply(ats.weight);
		        final_pos.add(new_pos);
				ats.initial_pos = bone.parent.model_pos.get();
				ats.initial_angle = bone.joint.angle;	 
			}
			set(final_pos.x(), final_pos.y(), final_pos.z());
		}
	}
	
	
	public static void execute(Utilities.CustomFrame model, ArrayList<Bone> bones){
	  	for(int i = 0; i < bones.size(); i++){
	  		bones.get(i).model_pos = model.coordinatesOf(bones.get(i).position().get());
	  		bones.get(i).prev_angle = bones.get(i).joint.angle;
	  		if(bones.get(i).parent == null) continue;
	  		bones.get(i).prev_pos = model.coordinatesOf(bones.get(i).parent.position().get());
	  	}		
		for(int i = 0; i < model.shape.getVertexCount(); i++){
			System.out.println("entra " + i);
			Vertex v = new Vertex(i, model);
			Vec vec = new Vec(v.v().x, v.v().y, v.v().z);
			Bone nearest = null;
			float n = 999999;
			for(Bone b : bones){
				System.out.println("entra " + n);
			    float rad = b.radiusX/model.magnitude();
				float[] dist =  Utilities.getDistance(vec, b);
				if(dist[1] < n){
					n = dist[1];
					nearest = b;
				}
				if(dist[1] <= rad ){
					v.addBone(b, dist[1]);
				}				
			}
			v.addBone(nearest, n);
			v.normalizeWeights();
			vertices.add(v);
		}
	}
	
	public static void applyTransformations(Utilities.CustomFrame model){
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
		for(Vertex v : vertices){
			v.applyTransformation();
		}
		
	}
}
