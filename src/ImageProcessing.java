import gab.opencv.*;
import processing.core.*;
import java.util.*;

public class ImageProcessing {
	//First Step get the image, and it contour
	//For this purpose we're gonna use OpenCV
	PImage source_image;
	PImage destination_image;
	PApplet parent;
	
	public ImageProcessing(PApplet p){
		parent = p;
	}

	//VARS TO FIND CONTOURS------------------------
	OpenCV opencv;
	float approximation = (float) 0.1;
	boolean invert = false; //change to true if the information of the image is in black 
	PShape figure;
	Utilities.Rectangle r_figure;
	ArrayList<Contour> contours        = new ArrayList<Contour>();
	Contour contour;
	ArrayList<PVector> edges           = new ArrayList<PVector>();  


	//COUNTOUR METHODS-------------------------------
	PShape getCountours(){
	  PShape img_contours;
	  opencv = new OpenCV(parent, source_image);
	  //convert to gray
	  opencv.gray();
	  //apply basic threshold
	  if(invert) opencv.invert();
	  opencv.threshold(10);
	  source_image = opencv.getOutput();
	  contours = opencv.findContours();  
	  //save just the external countour
	  contour = contours.get(0);
	  for (Contour c : contours){
	    contour = contour.numPoints() < c.numPoints() ? c : contour;
	  }
	  contour.setPolygonApproximationFactor(approximation);
	  contour = contour.getPolygonApproximation();

	  System.out.println("founded a contour with" + contour.numPoints() + " points");  
	  //save the points
	  edges = contour.getPoints();
	  img_contours = getCountoursShape((PImage)null);
	  return img_contours;
	}

	void getCountoursShape(PShape img_contours){
	  getCountoursShape((PImage)null);
	}

	PShape getCountoursShape(PImage text){
	  PShape figure = parent.createShape();
	  figure.beginShape();
	  if(text != null){
	    text.resize(Kinematics.all_width,2*Kinematics.all_height/3);
	    figure.textureMode(PConstants.IMAGE);    
	    figure.texture(text);
	  }
	  
	  for(int k = 0; k < edges.size();k++){
	    figure.stroke(255,255,255); 
	    figure.strokeWeight(1); 
	    figure.fill(parent.color(0,0,255,100));
	    figure.vertex(edges.get(k).x, edges.get(k).y,edges.get(k).x, edges.get(k).y);
	  }
	  figure.endShape(PConstants.CLOSE);
	  return figure;
	}

	void getContours(PShape s, ArrayList<PVector> points){
	  s = getContours(points, parent.color(0,255,0,100));
	}

	PShape getContours(ArrayList<PVector> points, int col){
	  PShape s = parent.createShape();
	  s.beginShape();
	  for(int k = 0; k < points.size();k++){
	    s.stroke(255,255,255); 
	    s.strokeWeight(2); 
	    s.fill(col);
	    s.vertex(points.get(k).x, points.get(k).y );
	  }
	  s.endShape(PConstants.CLOSE);
	  return s;
	}
	//END COUNTOUR METHODS---------------------------


	public void setupFigure(){
	  source_image = parent.loadImage("human6.png");  
	  source_image.resize(0,400);   
	  //get the countours
	  figure = getCountours();
	  r_figure = new Utilities.Rectangle();
	  r_figure.getBoundingBox(edges);  
	  //associate the shape with the original shape frame
	  Kinematics.original_fig = new Utilities.CustomModelFrame(Kinematics.main_scene, figure);
	  Kinematics.original_fig.translate(-r_figure.getCenterX()/2,-r_figure.getCenterY()/2);
	  Kinematics.original_fig.scale((float) 0.5);
	}
}
