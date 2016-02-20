package org.usfirst.frc.team1732.robot;

import java.util.ArrayList;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;
	
public class Robot extends SampleRobot {
	
	// Images
	Image frame;
	Image binaryFrame;
	int numberParticles;
	int session;
	double direction;
	AxisCamera camera;
	
	
	CANTalon left1; CANTalon left2; CANTalon left3;
	CANTalon right1; CANTalon right2; CANTalon right3;

	// Constants
	NIVision.Range GOAL_HUE_RANGE = new NIVision.Range(110, 130); // Default hue range for goal
	NIVision.Range GOAL_SAT_RANGE = new NIVision.Range(200, 255); // Default saturation range for goal
	NIVision.Range GOAL_VAL_RANGE = new NIVision.Range(270, 255); // Default value range for goal
	
	double AREA = 100;
	double RATIO = 1.428571; // Goal width = 20 in. / goal height = 12 in. = 1.428
	double RATIO_MIN = 1.2; // goal width = 20 in. / goal height = 12 in. = 1.428
	double RATIO_MAX = 1.6; // Goal width = 20 in. / goal height = 12 in. = 1.428
	double SCORE_MIN = 75.0; // Minimum score to be considered a goal
	double VIEW_ANGLE = 77.8; // View angle for camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 52 for HD3000 square, 60 for HD3000 640x480
	int particleLimit = 10;
	
	double min_speed = 0.2;
	double max_speed = 0.5;
	
	public Robot() {
		direction = 0.5;
		left1 = new CANTalon(11); left2 = new CANTalon(21); left3 = new CANTalon(22);
		right1 = new CANTalon(14); right2 = new CANTalon(12); right3 = new CANTalon(13);
		
		camera = new AxisCamera("10.99.99.9");
		
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_U8, 0);
		
		SmartDashboard.putBoolean("Capture?", false);
		SmartDashboard.putBoolean("binaryFrame?", false);

		SmartDashboard.putNumber("Goal hue min", GOAL_HUE_RANGE.minValue);
		SmartDashboard.putNumber("Goal hue max", GOAL_HUE_RANGE.maxValue);
		SmartDashboard.putNumber("Goal sat min", GOAL_SAT_RANGE.minValue);
		SmartDashboard.putNumber("Goal sat max", GOAL_SAT_RANGE.maxValue);
		SmartDashboard.putNumber("Goal val min", GOAL_VAL_RANGE.minValue);
		SmartDashboard.putNumber("Goal val max", GOAL_VAL_RANGE.maxValue);
		
		SmartDashboard.putNumber("Goal aspect min", RATIO_MIN);
		SmartDashboard.putNumber("Goal aspect max", RATIO_MAX);
		SmartDashboard.putNumber("Particle area min", AREA);
		
		SmartDashboard.putNumber("Direction", 0);
		SmartDashboard.putBoolean("do Aim?", false);
		
		SmartDashboard.putNumber("Particle Limit", particleLimit);
		SmartDashboard.putNumber("Max Speed", max_speed);
		SmartDashboard.putNumber("Min Speed", min_speed);
		
		CameraServer.getInstance().setQuality(25);
	}

	public void operatorControl() {
		while (isOperatorControl() && isEnabled()) {
			if (SmartDashboard.getBoolean("Capture?", false)) {
				// get the image and 
				//NIVision.IMAQdxGrab(session, frame, 1);
				camera.getImage(frame);
				// Update threshold values from SmartDashboard. For performance reasons it is recommended to remove
				// this after calibration is finished.
				GOAL_HUE_RANGE.minValue = (int) SmartDashboard.getNumber("Goal hue min", GOAL_HUE_RANGE.minValue);
				GOAL_HUE_RANGE.maxValue = (int) SmartDashboard.getNumber("Goal hue max", GOAL_HUE_RANGE.maxValue);
				GOAL_SAT_RANGE.minValue = (int) SmartDashboard.getNumber("Goal sat min", GOAL_SAT_RANGE.minValue);
				GOAL_SAT_RANGE.maxValue = (int) SmartDashboard.getNumber("Goal sat max", GOAL_SAT_RANGE.maxValue);
				GOAL_VAL_RANGE.minValue = (int) SmartDashboard.getNumber("Goal val min", GOAL_VAL_RANGE.minValue);
				GOAL_VAL_RANGE.maxValue = (int) SmartDashboard.getNumber("Goal val max", GOAL_VAL_RANGE.maxValue);
				min_speed = SmartDashboard.getNumber("Min Speed");
				max_speed = SmartDashboard.getNumber("Max Speed");
				RATIO_MIN = SmartDashboard.getNumber("Goal aspect min",   RATIO_MIN);
				RATIO_MAX = SmartDashboard.getNumber("Goal aspect max",   RATIO_MAX);
				AREA      = SmartDashboard.getNumber("Particle area min", AREA);

				
				particleLimit = (int) SmartDashboard.getNumber("Particle Limit", particleLimit);

				// Threshold the image looking for yellow (Goal color)
				NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSL, GOAL_HUE_RANGE, GOAL_SAT_RANGE, GOAL_VAL_RANGE);

				// Send particle count to dashboard
				int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
				SmartDashboard.putNumber("Masked particles", numParticles);

				if (numParticles > 0) {					
					// Measure particles and sort by particle size
					// Finds 15 largest particles
					
					ArrayList<Particle> qualifyingParticles = new ArrayList<Particle>();
					for (int particleIndex = 0; particleIndex < numParticles; particleIndex++) {
						Particle par = new Particle(NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP),
													NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT),
													NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM),
													NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT));
						
						double temp = par.getAspect();
						
						if (par.getArea() > AREA && temp < RATIO_MAX && temp > RATIO_MIN) {
							qualifyingParticles.add(par);
							
							//if (qualifyingParticles.size() > particleLimit) qualifyingParticles.remove(qualifyingParticles.size() - 1);
						}
					}
					
					SmartDashboard.putNumber("Filtered particles", qualifyingParticles.size());
					
					if(qualifyingParticles.size() > 0) {
						Particle bestPar = qualifyingParticles.get(0);
						
						for (int i = 1; i < qualifyingParticles.size(); i++)
							if (Math.abs(RATIO - qualifyingParticles.get(i).getAspect()) < Math.abs(RATIO - bestPar.getAspect()))
								bestPar = qualifyingParticles.get(i);
						direction = bestPar.getDirection();
						SmartDashboard.putNumber("Area", bestPar.getArea());
						SmartDashboard.putNumber("Left", bestPar.getLeft()/640.0);
						SmartDashboard.putNumber("Right", bestPar.getRight()/640.0);
						SmartDashboard.putNumber("Top", bestPar.getTop()/480.0);
						SmartDashboard.putNumber("Bottom", bestPar.getBottom()/480.0);
						SmartDashboard.putNumber("Aspect", bestPar.getAspect());
						SmartDashboard.putNumber("Distance",  bestPar.getDistance());
						SmartDashboard.putNumber("Direction", direction);
						if (SmartDashboard.getBoolean("do Aim?", false)) turn(direction, bestPar.getDistance());
						else setMotors(0,0);
					}
					else {
						if (SmartDashboard.getBoolean("do Aim?", false)) turn(direction, 100000000);
						else setMotors(0,0);
					}
					
				}
				
				// Send masked image to dashboard to assist in tweaking mask.
				if(SmartDashboard.getBoolean("binaryFrame?", false)) CameraServer.getInstance().setImage(binaryFrame);
				else CameraServer.getInstance().setImage(frame);
			}
		}
	}
	
	public void setMotors(double l, double r) {
		left1.set(l); left2.set(l); left3.set(-l); 
		right1.set(r); right2.set(r); right3.set(-r);
	}
	
	public double limit(double picasso) {
		return (picasso > 0 ? 1 : -1) * (Math.abs(picasso) < min_speed ? min_speed : (Math.abs(picasso) > max_speed ? max_speed : Math.abs(picasso)));
	}
	
	public void turn(double dir, double dist) {
		//double dir = par.getDirection();
		if (Math.abs(dir - 0.5) < 0.2) {
			setMotors((dist-200)/10.0, (dist-200)/10.0);
		} else {
			setMotors(limit((dir - 0.5) / 2.0), limit((dir - 0.5) / 2.0));
		}
		
		
		//if (dir < .4) setMotors(-0.13, -0.13);
		//else if (dir > .6) setMotors(0.13, 0.13);
		//else setMotors(0,0);
	}
	
	
	/*public static int sortParticles(Particle newPar, ArrayList<Particle> qualifyingParticles) {
		for (int i = 0; i < qualifyingParticles.size(); i++) {
			if (newPar.getArea() > qualifyingParticles.get(i).getArea()) {
				return i;
			}
		} return -1;
	}
	
	// Simple method that finds the particles with the highest areas
	// No need to worry about what happens if the original size is filled up- there are only set methods, not add methods used

	/* Comparator function for sorting particles. Returns true if particle 1 is larger
	static boolean CompareParticleSizes(ParticleReport particle1, ParticleReport particle2) {
		// we want descending sort order
		return particle1.PercentAreaToImageArea > particle2.PercentAreaToImageArea;
	}

	/*//*
	 * Method to score if the aspect ratio of the particle appears to match the
	 * retro-reflective target. Target is 7"x7" so aspect should be 1
	 ///
	double AspectScore(ParticleReport report) {
		return ratioToScore(((report.BoundingRectRight - report.BoundingRectLeft) / (report.BoundingRectBottom - report.BoundingRectTop)));
	}

	/*//*
	 * Computes the estimated distance to a target using the width of the
	 * particle in the image. For more information and graphics showing the math
	 * behind this approach see the Vision Processing section of the
	 * ScreenStepsLive documentation.
	 *
	 * @param image
	 *            The image to use for measuring the particle estimated
	 *            rectangle
	 * @param report
	 *            The Particle Analysis Report for the particle
	 * @param isLong
	 *            Boolean indicating if the target is believed to be the long
	 *            side of a tote
	 * @return The estimated distance to the target in feet.
	 ///
	double computeDistance(Image image, ParticleReport report) {
		double normalizedWidth, targetWidth;
		NIVision.GetImageSizeResult size;

		size = NIVision.imaqGetImageSize(image);
		// I don't understand what normalized width is supposed to do, it just seems like it will screw up the width of the particle
		// like if right = 200, left = 0, width = 300, normalized width = 4/9, not 2/3!!!!!!
		normalizedWidth = 2 * (report.BoundingRectRight - report.BoundingRectLeft) / size.width; // a constant, can be replaced/delete above code
		targetWidth = 7;
		// Need to change some of these values because they don't match up with the goal.
		return targetWidth / (normalizedWidth * 12 * Math.tan(VIEW_ANGLE * Math.PI / (180 * 2)));
	}//*/

}