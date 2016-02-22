package org.usfirst.frc.team1732.robot;

import java.util.Comparator;

public class Particle implements Comparable<Particle> {
	
	// Image width:  640
	//		 height: 480
	
	private double left;
	private double top;
	private double right;
	private double bottom;

	public static Comparator<Particle> ParticleComparator = new Comparator<Particle>() {
		public int compare(Particle p1, Particle p2) {
			return p1.compareTo(p2);
		}
	};
	
	Particle(double top, double left, double bottom, double right) {
		this.top = top;
		this.left = left;
		this.bottom = bottom;
		this.right = right;
	}
	
	public double getLeft() {return left;}
	public double getRight() {return right;}
	public double getTop() {return top;}
	public double getBottom() {return bottom;}
	
	public double getArea() {
		return Math.abs((right-left)*(bottom-top));
	}
	
	public double getAspect() {
		return (Math.abs(right-left))/(Math.abs(bottom-top));
	}

	public int compareTo(Particle par) {
		return (int) (((Particle) par).getArea() - (this.getArea()) * 100);
	}
	
	public double getDistance() {
		return (640*20) / (Math.abs(right-left)*2*Math.tan(Math.toRadians(47/2)));
	}
	// 640/w = 2*tan(47/2)*d/20
	
	public double getDirection() {
		return (right+left)/1280;
	}

}