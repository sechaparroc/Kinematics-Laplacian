import java.util.*;

import processing.core.*;
import remixlab.bias.event.*;
import remixlab.dandelion.core.*;
import remixlab.dandelion.geom.*;
import remixlab.proscene.*;
import smile.math.Math;

public class Utilities{
	public static class Rectangle{
		  float x,y,w,h;  
		  public Rectangle(){
			  
		  }
		  public Rectangle(float xx, float yy, float ww, float hh){
		    x = xx;
		    y = yy;
		    w = ww; 
		    h= hh; 
		  }
		  float getCenterX(){
		    return x + w/2;
		  }
		  float getCenterY(){
		    return y + h/2;
		  }
		  @Override
		  public String toString(){
		    String s = "Rectangle: \n";
		    s += "UL : x = " + x + ", y = " + y; 
		    s += "width = " + w + ", height = " + h; 
		    s += "centerX = " + getCenterX() + ", centerY = " + getCenterY(); 
		    return s;
		  }
		  
		public void getBoundingBox(ArrayList<PVector> points){
			  PVector top = new PVector(9999,9999);
			  PVector bottom = new PVector(-9999,-9999);
			  for(PVector p : points){
			    if(p.x < top.x) top.x = p.x;  
			    if(p.y < top.y) top.y = p.y;  
			    if(p.x > bottom.x) bottom.x = p.x;  
			    if(p.y > bottom.y) bottom.y = p.y;  
			  }
			  this.x = top.x;
			  this.y = top.y;
			  this.w = bottom.x - top.x;
			  this.h = bottom.y - top.y;
			} 

			public void getBoundingBox(PShape sh){
			  PVector top = new PVector(9999,9999);
			  PVector bottom = new PVector(-9999,-9999);
			  for(int j = 0; j < sh.getChildCount(); j++){
			    PShape shc = sh.getChild(j);
			    for(int i = 0; i < shc.getVertexCount(); i++){
			      PVector p = shc.getVertex(i);
			      if(p.x < top.x) top.x = p.x;  
			      if(p.y < top.y) top.y = p.y;  
			      if(p.x > bottom.x) bottom.x = p.x;  
			      if(p.y > bottom.y) bottom.y = p.y;    
			    }
			  }
			  this.x = top.x;
			  this.y = top.y;
			  this.w = bottom.x - top.x;
			  this.h = bottom.y - top.y;
			} 
			//-----------------------------------------------		  
		}

		//UTIL ALGORITHMS--------------------------------
		public static ArrayList<PVector> quickSort(ArrayList<PVector> list, PVector comp, int size){
		  if(size < 2) return list;
		  Random rand = new Random();
		  int pivot = rand.nextInt(size);
		  int p1 = 0,p2 = 0;
		  ArrayList<PVector>list1 = new ArrayList<PVector>();
		  ArrayList<PVector>list2 = new ArrayList<PVector>();  
		  //reorganize list
		  for(int k = 0; k < size; k++){
		    if(list.get(k).dist(comp) < list.get(pivot).dist(comp)){
		      list1.add(list.get(k));
		      p1++;
		    }else{
		      if(k != pivot){
		        list2.add(list.get(k));
		        p2++;
		      }
		    }
		  }
		  //recursion
		  list1 = quickSort(list1, comp, p1);
		  list2 = quickSort(list2, comp, p2);
		  PVector num_pivot = list.get(pivot);
		  //return the list in the right order
		  for(int k = 0; k < p1; k++){
		    list.set(k,list1.get(k));
		  }
		  list.set(p1, num_pivot);
		  for(int k = 0; k < p2; k++){
		    list.set(p1 + k + 1, list2.get(k));
		  }
		  return list;
		}
		
		public static class CustomFrame extends GenericP5Frame{
		  PShape shape;
		  public CustomFrame(Scene sc, PShape s){
		    super(sc);
		    shape = s;
		  }		  
		  public PShape shape(){
		    return shape;
		  }		  
		  public void draw(){
		    scene().pg().pushMatrix();
		    //root.applyWorldTransformation();
		    scene().applyWorldTransformation(this);
		    //scene().drawAxes(40);    
		    scene().pg().shape(shape);    
		    scene().pg().popMatrix();
		  }		    
		}
		
	  public static float[] getDistance(Vec vv, Bone b){
		    if(b.parent == null) return new float[]{99999,9999};
		    //is the distance btwn line formed by b and its parent and v
	    Vec line = Vec.subtract(b.model_pos, b.parent.model_pos);
	    Vec va = Vec.subtract(vv, b.parent.model_pos);
	    float dot = Vec.dot(va, line);
	    float mag = line.magnitude();
	    float u  = dot*(float)1./(mag*mag);
	    Vec aux = new Vec();
	    if(u >= 0 && u <=1){
	      aux = new Vec(b.parent.model_pos.x() + u*line.x(), b.parent.model_pos.y() + u*line.y()); 
	      aux = Vec.subtract(aux, vv);
	    }
	    if(u < 0){
	      aux = Vec.subtract(b.parent.model_pos, vv); 
	    }
	    if(u > 1){
	      aux = Vec.subtract(b.model_pos, vv); 
	    }
	    return new float[]{u, aux.magnitude()};
	  }
}
