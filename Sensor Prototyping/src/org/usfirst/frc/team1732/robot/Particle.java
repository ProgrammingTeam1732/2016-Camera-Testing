package org.usfirst.frc.team1732.robot;

public class Particle {
	
	private double left;
	private double top;
	private double right;
	private double bottom;
	
	Particle(double top, double left, double bottom, double right) {
		this.top = top;
		this.left = left;
		this.bottom = bottom;
		this.right = right;
	}
	
	public double getLeft() {
		return left;
	}
	
	public double getRight() {
		return right;
	}
	
	public double getTop() {
		return top;
	}
	
	public double getBottom() {
		return bottom;
	}
	
}
