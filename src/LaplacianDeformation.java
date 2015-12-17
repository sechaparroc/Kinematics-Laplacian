import java.util.ArrayList;
import java.util.HashMap;

import processing.core.PShape;
import processing.core.PVector;
import remixlab.dandelion.core.MatrixHelper;
import remixlab.dandelion.geom.Vec;
import smile.data.Dataset;
import smile.data.SparseDataset;
import smile.data.parser.SparseDatasetParser;
import smile.math.Math;
import smile.math.SparseArray;
import smile.math.matrix.CholeskyDecomposition;
import smile.math.matrix.IMatrix;
import smile.math.matrix.LUDecomposition;
import smile.math.matrix.Matrix;
import smile.math.matrix.QRDecomposition;
import smile.math.matrix.SparseMatrix;
import smile.util.SmileUtils;

/*
 * November 19 2015
 * Sebastian Chaparro
 * 
 * This is an implementation of Laplacian Surface Deformation 
 * The paper can be found at: 
 * http://igl.ethz.ch/projects/Laplacian-mesh-processing/Laplacian-mesh-editing/laplacian-mesh-editing.pdf
 * 
 * */

public class LaplacianDeformation {
	public ArrayList<Vertex> vertices;
	public ArrayList<Anchor> anchors;
	public SparseDataset A, L, M; 
	static boolean debug = false;
	
	public class Anchor{
		public class AnchorAttribs{
			Bone related_bone;			
			PVector initial_pos;			
			float weight;
			float initial_angle;
			PVector pos;
		}
		float weight = 0;
		ArrayList<AnchorAttribs> attribs;
		Vertex vertex;
		PVector pos;
		int idx;//id of the control point
		
		public Anchor(Bone b, Vertex vv, int ii, PVector p_model, float weight){
			attribs = new ArrayList<AnchorAttribs>();
			AnchorAttribs ats = new AnchorAttribs();
			ats.related_bone = b;
			vertex = vv;
			pos = vv.v;
			idx = ii;
			ats.initial_angle = b.joint.angle;
			ats.initial_pos = p_model;
			ats.weight = weight;
			attribs.add(ats);
		}

		public Anchor(Vertex vv, int ii){
			vertex = vv;
			idx = ii;
		}

		public void addAttrib(Bone b, PVector pVector, float weight) {
			AnchorAttribs ats =	new AnchorAttribs();
			ats.related_bone = b;
			ats.initial_angle = b.joint.angle;
			ats.initial_pos = pVector;
			ats.weight = weight;
			attribs.add(ats);
		}
	}
	
	public class Vertex{
		PVector v;
		PVector d;
		int idx;//row position in the adjacency matrix
		int idx_shape;//position in the shape to modify
		ArrayList<Vertex> neighbors;		

		public Vertex(PVector vv, int i, int is){
			v = vv;
			idx = i;
			idx_shape = is;
			neighbors = new ArrayList<Vertex>();			
		}
		
		public void addNeighbor(Vertex n){
			neighbors.add(n);
			if(n.neighbors.contains(this) == false) n.neighbors.add(this);
		}
	}
	
	public void addEdge(SparseDataset A, Vertex v1, Vertex v2){
		//The whole vetex is used as arg if its desired to use other weight scheme
		A.set(v1.idx, v2.idx, 1);
		A.set(v2.idx, v1.idx, 1);		
	}
	
	public void setup(PShape shape){
		getNeighbors(shape);
		getLaplacian();
		anchors = new ArrayList<Anchor>();
	}
	
	public void setup(SparseDataset O, ArrayList<Vertex> vs){
		getSubregionNeighbors(O, vs);
		getLaplacian();
		anchors = new ArrayList<Anchor>();
	}
	
	public Vertex getVertex(PVector v){
		for(Vertex v_i : vertices){
			if(v_i.v == v) return v_i;
			
		}
		return null;
	}
		
	public void getSubregionNeighbors(SparseDataset O, ArrayList<Vertex> vs){
		A = new SparseDataset();
		vertices = new ArrayList<Vertex>();
		//add the vertices
		int idx = 0;
		for(Vertex v_i : vs){
			Vertex v = new Vertex(v_i.v, -1, v_i.idx_shape);
			vertices.add(v);
			for(Vertex v_j : v_i.neighbors){
				Vertex aux = getVertex(v_j.v); 
				if(aux != null){
					v.addNeighbor(aux);
				}
			}
		}
		ArrayList<Vertex> border = new ArrayList<Vertex>();
		for(int i = 0; i < vertices.size(); i++){
			if(vertices.get(i).neighbors.isEmpty()){
				vertices.remove(i--);
			}
			else if(vertices.get(i).neighbors.size() == 1 && !border.contains(vertices.get(i))){
				border.add(vertices.get(i));
				border.add(getOtherBorder(vertices.get(i)));
			}
		}
		//join unnconected points
		//case A: single border
		if(border.size() == 2){
			Vertex v1 = border.remove(0);
			Vertex v2 = border.remove(0);
			v1.addNeighbor(v2);			
		}
		//case B: more than one border
		if(border.size() == 4){
			Vertex v1 = border.remove(0);
			Vertex v2 = border.remove(0);
			Vertex v3 = getNearest(border, v1.v);
			border.remove(v3);
			Vertex v4 = border.remove(0);
			v1.addNeighbor(v3);			
			v2.addNeighbor(v4);			
		}
		
		idx = 0;		
		for(Vertex v_i : vertices){
			v_i.idx = idx;
			for(Vertex v_j : v_i.neighbors){
				if(v_j.idx != -1)addEdge(A, v_i, v_j);
			}
			idx++;
		}
	}
	
	public void getNeighbors(PShape shape){
		A = new SparseDataset();
		vertices = new ArrayList<Vertex>();
		Vertex prev = new Vertex(shape.getVertex(0),0,0);
		vertices.add(prev);
		Vertex v0 = prev;
		for(int i = 1; i < shape.getVertexCount(); i++){
			PVector vec = shape.getVertex(i);
			Vertex v = new Vertex(vec,i,i);
			v.addNeighbor(prev);
			addEdge(A,v,prev);
			prev = v;
			vertices.add(prev);
		}
		addEdge(A,v0,prev);
		prev.addNeighbor(v0);
	}

	public void getLaplacian(){
		int n = vertices.size();
		//M is used as the matrix to get the new positions of the vertices
		M = new SparseDataset();
		L = new SparseDataset();
		for(Vertex v_i : vertices){
			double dx = v_i.v.x;
			double dy = v_i.v.y;
			L.set(v_i.idx, v_i.idx, 1);
			M.set(v_i.idx, v_i.idx, 1);
			M.set(v_i.idx + n, v_i.idx + n, 1);
			int degree = v_i.neighbors.size();
			for(Vertex v_j : v_i.neighbors){
				L.set(v_i.idx, v_j.idx, -1./degree);
				dx += -(1./degree) * v_j.v.x;
				dy += -(1./degree) * v_j.v.y;
				M.set(v_i.idx, v_j.idx, -1./degree);
				M.set(v_i.idx + n, v_j.idx + n, -1./degree);				
			}
			v_i.d = new PVector((float)dx, (float)dy);
		}
		//printMat("Laplacian", L.toArray());
		//printMat("Initial M", M.toArray());		
	}

	public Vertex getNearest(ArrayList<Vertex> vertices, PVector p){
		float min_dist = 99999;
		Vertex min = null;
		for(Vertex v : vertices){
			if(PVector.dist(v.v, p) < min_dist){
				min = v;
				min_dist = PVector.dist(v.v, p);
			}
		}
		return min;
	}
	
	public Vertex getFarthest(ArrayList<Vertex> vertices, PVector p){
		float max_dist = -99999;
		Vertex max = null;
		for(Vertex v : vertices){
			if(PVector.dist(v.v, p) > max_dist){
				max = v;
				max_dist = PVector.dist(v.v, p);
			}
		}
		return max;
	}

	
	public Vertex getOtherBorder(Vertex v){
		//assume that vertex V is in the border
		Vertex prev = v;
		Vertex next = v.neighbors.get(0);
		while(next.neighbors.size() > 1){
			if(debug)System.out.println("varado");
			Vertex n = next.neighbors.get(0) == prev ? next.neighbors.get(1) : next.neighbors.get(0);
			prev = next;
			next = n;
		}
		return next;
	}
	
	
	public Vertex getNearest(PVector p){
		return getNearest(this.vertices, p);
	}

	public Vertex getFarthest(PVector p){
		return getFarthest(this.vertices, p);
	}
	
	public Anchor containsAnchor(ArrayList<Anchor> anchors, Vertex v){
		for(Anchor a : anchors){
			if(a.vertex == v) return a;
		}
		return null;
	}

		
	public void addAnchorByDist(ArrayList<Anchor> anchors, Utilities.CustomModelFrame model, Bone b, int i, float percentage){
		PVector p2 = new PVector(b.model_pos.x(), b.model_pos.y());
		PVector p1 = new PVector(b.parent.model_pos.x(), b.parent.model_pos.y());
		PVector p_mid = PVector.add(p1, p2);
		p_mid.mult(0.5f);
		//percentage is the total of vertices to add to the anchor
		int num_anchors = (int) (percentage * vertices.size());
		int s1 = 0, s2 = 0, s3 = 0;
		if(debug)System.out.println("num anchors %%%%" + num_anchors + "num vert" + vertices.size());
		ArrayList<Vertex> sorted_p1 = sortByDistance(vertices, p1);
		ArrayList<Vertex> sorted_p2 = sortByDistance(vertices, p2);
		ArrayList<Vertex> sorted_p3 = sortByDistance(vertices, p_mid);
		ArrayList<Vertex> sorted = new ArrayList<Vertex>();
		for(int c = 0; c < num_anchors/3; c++){
			sorted.add(sorted_p1.get(c));
			s1++;
		}
		for(int c = 0; c < num_anchors/3; c++){
			if(!sorted.contains(sorted_p2.get(c))){
				sorted.add(sorted_p2.get(c));
				s2++;
			}
		}
		for(int c = 0; c < num_anchors/3; c++){
			if(!sorted.contains(sorted_p3.get(c))){
				sorted.add(sorted_p3.get(c));
				s3++;
			}
		}
		//add also points in the middle
		
		Vertex n = getNearest(sorted, p1);
		Vertex f = getFarthest(sorted, p1);		
		float nearest = PVector.dist(n.v, p1);
		float farthest = PVector.dist(f.v, p1);
		for(Vertex v : sorted){
			float weight = Math.abs((farthest - PVector.dist(p1, v.v)))/Math.abs((farthest - nearest ));
			if(debug)System.out.println("################# <<<<<<<<<<<<<---- : n1 " + nearest);
			if(debug)System.out.println("################# <<<<<<<<<<<<<---- : f1 " + farthest);
			if(debug)System.out.println("################# <<<<<<<<<<<<<---- : n2 " + nearest);			
			if(debug)System.out.println("################# <<<<<<<<<<<<<---- : f2 " + farthest);			
			if(debug)System.out.println("################# <<<<<<<<<<<<<---- : d " + PVector.dist(p1, v.v));			
			if(debug)System.out.println("################# <<<<<<<<<<<<<---- : weight " + weight);
			Vec initial = model.coordinatesOf(b.parent.position().get());
			Anchor anchor = new Anchor(b, v, i, new PVector(initial.x(), initial.y()), 0);
			Anchor a = containsAnchor(this.anchors, v);
			if(a == null){
				anchors.add(anchor);
			}else{
				a.addAttrib(b,new PVector(initial.x(), initial.y()),0);
			}			
		}
	}
		
	public void calculateLaplacian(){
		int n = vertices.size();	
		if(debug)printMat("laplacian",L.toArray(),30,30);
		for(Vertex v_i : vertices){
			int num_n = v_i.neighbors.size();			
			double[][] T_data = new double[(num_n+1)*2][4];
			int idx = 0;
			T_data[idx] = new double[]{v_i.v.x,  v_i.v.y, 1, 0};
			T_data[idx + num_n + 1] = new double[]{v_i.v.y, -v_i.v.x, 0, 1};			
			idx++;
			for(Vertex v_j : v_i.neighbors){
				T_data[idx] = new double[]{v_j.v.x,  v_j.v.y, 1, 0};
				T_data[idx + num_n + 1] = new double[]{v_j.v.y, -v_j.v.x, 0, 1};			
				idx++;
			}
			
			QRDecomposition qr = new QRDecomposition(T_data);
			//Matrix T = new Matrix(T_data);
			//qr.inverse();
			double[][] T_inv = new double[4][(num_n+1)*2];
			qr.solve(Math.eye((num_n+1)*2, (num_n+1)*2), T_inv);
			if(debug)printMat("inverse T implicit",T_inv);
			
			//get the linear transformation coefficients
			double[] s = T_inv[0];
			double[] a = T_inv[1];
			//s = new double[]{-1,0,0,-1,0,0};
			//a = new double[]{0,-1,0,0,-1,0};

			//apply the transformation to laplacian coords
			double[][] T_delta = new double[2][(num_n+1)*2];
			for(int i = 0; i < T_delta[0].length; i++){
				T_delta[0][i] =  s[i]*v_i.d.x + a[i]*v_i.d.y;
				T_delta[1][i] = -a[i]*v_i.d.x + s[i]*v_i.d.y;				
			}
			if(debug)printMat("T delta",T_delta);
			//Update values on M
			idx = 0;
			M.set(v_i.idx    , v_i.idx    , M.get(v_i.idx    , v_i.idx    ) - T_delta[0][idx]);
			M.set(v_i.idx + n, v_i.idx    , M.get(v_i.idx + n, v_i.idx    ) - T_delta[1][idx]);
			M.set(v_i.idx    , v_i.idx + n, M.get(v_i.idx    , v_i.idx + n) - T_delta[0][idx + num_n + 1]);
			M.set(v_i.idx + n, v_i.idx + n, M.get(v_i.idx + n, v_i.idx + n) - T_delta[1][idx + num_n + 1]);
			idx++;
			for(Vertex v_j : v_i.neighbors){
				M.set(v_i.idx    , v_j.idx    , M.get(v_i.idx    , v_j.idx    ) - T_delta[0][idx]);
				M.set(v_i.idx + n, v_j.idx    , M.get(v_i.idx + n, v_j.idx    ) - T_delta[1][idx]);
				M.set(v_i.idx    , v_j.idx + n, M.get(v_i.idx    , v_j.idx + n) - T_delta[0][idx + num_n + 1]);
				M.set(v_i.idx + n, v_j.idx + n, M.get(v_i.idx + n, v_j.idx + n) - T_delta[1][idx + num_n + 1]);
				idx++;
			}
		}
	}		
	
	public ArrayList<PVector> solveLaplacian(){
		int n = vertices.size();			
		SparseDataset M = new SparseDataset();
		SparseDataset M_T = new SparseDataset();
		for(int i = 0; i < this.M.size(); i++){
			for(int j = 0; j < this.M.ncols(); j++){
				double val = this.M.get(i, j);
				M.set(i,j,val);
				M_T.set(j,i,val);					
			}
		}
		
		ArrayList<Anchor> anchors = new ArrayList<Anchor>();
		anchors.addAll(this.anchors);
		int m_dim = M.size();
		if(debug)System.out.println("m_dim : " + m_dim);		
		
		double[] RHS = new double[m_dim + 2*anchors.size()];		
		
		for(Anchor anchor : anchors){
			M.set(m_dim, anchor.vertex.idx, anchor.weight);
			M_T.set(anchor.vertex.idx, m_dim, anchor.weight);
			RHS[m_dim++] = anchor.weight*anchor.pos.x;
			//System.out.println("--> RHS : " + "( " + RHS[m_dim-1] + ", " + (int)(m_dim-1) + " ) ");
			M.set(m_dim, anchor.vertex.idx + n, anchor.weight);
			M_T.set(anchor.vertex.idx + n, m_dim, anchor.weight);
			RHS[m_dim++] = anchor.weight*anchor.pos.y;
			//System.out.println("--> RHS : " + "( " + RHS[m_dim-1] + ", " + (int)(m_dim-1) + " ) ");
		}
		if(debug)printMat("yo q se : ", M.toArray());

		//Solve
		SparseMatrix MMT = M.toSparseMatrix().transpose().times(M.toSparseMatrix()); 
//				SparseMatrix.AAT(M.toSparseMatrix(), M.toSparseMatrix().transpose());
		//System.out.println("m_dim : " + m_dim);		
		//System.out.println("MMT DIMS : " + MMT.nrows() + " cols : " + MMT.ncols());		
		//System.out.println("M DIMS : " + M.size() + " cols : " + M.ncols());		
		//System.out.println("MT DIMS : " + M.toSparseMatrix().transpose().nrows() + " cols : " + M.toSparseMatrix().transpose().ncols());		

		Matrix LHS = new Matrix(matrixToArray(MMT), true, true);
		Matrix M_aux = new Matrix(M_T.toArray());		
		//double 
		double[] RHSS = new double[M_aux.nrows()];
		M_aux.ax(RHS, RHSS);
		if(debug)printArr("rhs " + RHS.length, RHS);
		if(debug)printArr("new rhs", RHSS);
		if(debug)printMat("m cond", M.toArray());
		if(debug)printMat("m trans", M_T.toArray());

		double[] new_coords = new double[LHS.ncols()];	
		CholeskyDecomposition ch = LHS.cholesky();
		//QRDecomposition ch = new QRDecomposition(M.toArray());
		ch.solve(RHSS, new_coords);		
		ArrayList<PVector> new_img = new ArrayList<PVector>();
		for(int i = 0; i < n; i++){
			//System.out.println("--> prev_coord : " + "( " + vertices.get(i).v.x + ", " + vertices.get(i).v.y + " ) ");
			//System.out.println("--> coord : " + "( " + new_coords[i] + ", " + new_coords[i+n] + " ) ");
			new_img.add(new PVector((float)new_coords[i], (float)new_coords[i+n]));
		}
		/*for(int i = 0; i < RHS.length; i++){
			System.out.println("--> RHS : " + "( " + RHS[i] + ", " + i + " ) ");
		}*/

		//printArr("RHS", RHS);
		//printArr("coords", new_coords);
		//System.out.println("previous coords");
		//for(int i = 0; i < n; i++){
		//	System.out.print("(" + vertices.get(i).v.x + ", " + vertices.get(i).v.y + "), ");
		//}		
		//for(int i = 0; i < n; i++){
			//System.out.print(vertices.get(i).v.y + ", ");
		//}		
		return new_img;		
	}
	
	public ArrayList<Vertex> sortByDistance(ArrayList<Vertex> vertices, PVector p){
		if(vertices.size() == 1) return vertices;
		if(vertices.size() == 0) return vertices;
		int p_idx = vertices.size()/2;
		Vertex pivot = vertices.get(p_idx); 
		float dist_p = PVector.dist(pivot.v, p);
		//divide
		ArrayList<Vertex> leftside = new ArrayList<Vertex>();
		ArrayList<Vertex> rightside = new ArrayList<Vertex>();
		for(Vertex v : vertices){
			if(v == pivot) continue;
			float dist = PVector.dist(v.v, p);
			if(dist < dist_p){
				leftside.add(v);
			}else{
				rightside.add(v);
			}
		}
		//sort the sublists
		leftside = sortByDistance(leftside, p);
		rightside = sortByDistance(rightside, p);
		//join the sublists
		ArrayList<Vertex> sorted = new ArrayList<Vertex>();
		sorted.addAll(leftside);
		sorted.add(pivot);
		sorted.addAll(rightside);		
		return sorted;
	}
	
	
	static void printMat(String name, double[][] m){
		System.out.println("---------------------");
		System.out.println(name);		
		System.out.println("---------------------");		
		for(int i = 0; i < m.length; i++){
			for(int j = 0; j < m[0].length; j++){
				System.out.printf("%.2f" + ", " , m[i][j]);
			}
			System.out.println();
		}
		System.out.println("---------------------");		
		System.out.println("---------------------");		
	}

	static void printMat(String name, double[][] m, int r, int c){
		System.out.println("---------------------");
		System.out.println(name);		
		System.out.println("---------------------");
		int rr = Math.min(r, m.length);
		int cc = Math.min(c, m[0].length);		
		for(int i = 0; i < rr; i++){
			for(int j = 0; j < cc; j++){
				System.out.printf("%.2f" + ", \t" , m[i][j]);
			}
			System.out.println();
		}
		System.out.println("---------------------");		
		System.out.println("---------------------");		
	}
	
	
	static void printArr(String name, double[] m){
		System.out.println("---------------------");
		System.out.println(name);		
		System.out.println("---------------------");		
		for(int i = 0; i < m.length; i++){
				System.out.printf("%.2f" + ", " , m[i]);
		}
		System.out.println("---------------------");		
		System.out.println("---------------------");		
	}
	
	static double[][] matrixMult(double[][] m1, double[][] m2){
		int n = m1.length;
		int m = m1[0].length;
		int l = m2[0].length;
		double[][] result = new double[n][l];
		for(int i = 0; i < n; i++){
			for(int k = 0; k < m; k++){
				for(int j = 0; j < l; j++){
					result[i][j] += m1[i][j] * m2[j][k]; 
				}			
			}
		}
		return result;
	}
	
	static double[][] matrixToArray(IMatrix m){
		double[][] result = new double[m.nrows()][m.ncols()];
		for(int i = 0; i < m.nrows(); i++){
			for(int j = 0; j < m.ncols(); j++){
				result[i][j] = m.get(i, j);	
			}
		}
		return result;
	}
}
