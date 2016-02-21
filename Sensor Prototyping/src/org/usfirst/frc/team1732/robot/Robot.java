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
	
	// Camera/Images
	Image frame;
	Image binaryFrame;
	int numberParticles;
	AxisCamera camera;
	
	// Motors
	CANTalon left1; CANTalon left2; CANTalon left3;
	CANTalon right1; CANTalon right2; CANTalon right3;

	// Constants
	double VIEW_ANGLE = 77.8;
	double RATIO = 1.428571; // Goal width = 20 in. / goal height = 12 in. = 1.428
	
	// Color Limits
	NIVision.Range PAR_HUE_RANGE = new NIVision.Range(110, 130); // Default hue range for goal
	NIVision.Range PAR_SAT_RANGE = new NIVision.Range(200, 255); // Default saturation range for goal
	NIVision.Range PAR_VAL_RANGE = new NIVision.Range(270, 255); // Default value range for goal
	
	// Search Limits
	double RATIO_MIN = 1.2; // goal width = 20 in. / goal height = 12 in. = 1.428
	double RATIO_MAX = 1.6; // Goal width = 20 in. / goal height = 12 in. = 1.428
	double MIN_AREA = 100;
	int PAR_LIMIT = 10;
	
	// Driving
	double min_speed = 0.2;
	double max_speed = 0.5;
	double direction = 0.5;
	
	public Robot() {
		// Motors
		left1 = new CANTalon(11); left2 = new CANTalon(21); left3 = new CANTalon(22);
		right1 = new CANTalon(14); right2 = new CANTalon(12); right3 = new CANTalon(13);
		
		// Camera/Images
		camera = new AxisCamera("10.99.99.9");
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_HSL, 0);
		binaryFrame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_U8, 0);
		CameraServer.getInstance().setQuality(25);
		
		// Camera
		SmartDashboard.putBoolean("Capture?", false);
		SmartDashboard.putBoolean("binaryFrame?", false);

		// Color limits
		SmartDashboard.putNumber("Goal hue min", PAR_HUE_RANGE.minValue);
		SmartDashboard.putNumber("Goal hue max", PAR_HUE_RANGE.maxValue);
		SmartDashboard.putNumber("Goal sat min", PAR_SAT_RANGE.minValue);
		SmartDashboard.putNumber("Goal sat max", PAR_SAT_RANGE.maxValue);
		SmartDashboard.putNumber("Goal val min", PAR_VAL_RANGE.minValue);
		SmartDashboard.putNumber("Goal val max", PAR_VAL_RANGE.maxValue);
		
		// Search limits
		SmartDashboard.putNumber("Goal aspect min", RATIO_MIN);
		SmartDashboard.putNumber("Goal aspect max", RATIO_MAX);
		SmartDashboard.putNumber("Particle area min", MIN_AREA);
		SmartDashboard.putNumber("Particle Limit", PAR_LIMIT);
		
		// Driving
		SmartDashboard.putNumber("Direction", 0);
		SmartDashboard.putBoolean("do Aim?", false);
		SmartDashboard.putNumber("Max Speed", max_speed);
		SmartDashboard.putNumber("Min Speed", min_speed);
		
		// Particle Display
		SmartDashboard.putNumber("Masked particles", 0);
		SmartDashboard.putNumber("Filtered particles", 0);
		SmartDashboard.putNumber("Area", 0);
		SmartDashboard.putNumber("Left", 0);
		SmartDashboard.putNumber("Right", 0);
		SmartDashboard.putNumber("Top", 0);
		SmartDashboard.putNumber("Bottom", 0);
		SmartDashboard.putNumber("Aspect", 0);
		SmartDashboard.putNumber("Distance",  0);
		SmartDashboard.putNumber("Direction", 0);
	}

	public void operatorControl() {
		while (isOperatorControl() && isEnabled()) {
			if (SmartDashboard.getBoolean("Capture?", false)) {
				camera.getImage(frame);
				// Set color limits
				PAR_HUE_RANGE.minValue = (int) SmartDashboard.getNumber("Particle hue min", PAR_HUE_RANGE.minValue);
				PAR_HUE_RANGE.maxValue = (int) SmartDashboard.getNumber("Particle hue max", PAR_HUE_RANGE.maxValue);
				PAR_SAT_RANGE.minValue = (int) SmartDashboard.getNumber("Particle sat min", PAR_SAT_RANGE.minValue);
				PAR_SAT_RANGE.maxValue = (int) SmartDashboard.getNumber("Particle sat max", PAR_SAT_RANGE.maxValue);
				PAR_VAL_RANGE.minValue = (int) SmartDashboard.getNumber("Particle val min", PAR_VAL_RANGE.minValue);
				PAR_VAL_RANGE.maxValue = (int) SmartDashboard.getNumber("Particle val max", PAR_VAL_RANGE.maxValue);

				// Threshold the image looking for yellow (Goal color)
				NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSL, PAR_HUE_RANGE, PAR_SAT_RANGE, PAR_VAL_RANGE);

				// Count and display particles
				int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
				SmartDashboard.putNumber("Masked particles", numParticles);
				SmartDashboard.putNumber("Filtered particles", 0);

				// Set search limits
				RATIO_MIN = SmartDashboard.getNumber("Particle aspect min",   RATIO_MIN);
				RATIO_MAX = SmartDashboard.getNumber("Particle aspect max",   RATIO_MAX);
				MIN_AREA  = SmartDashboard.getNumber("Particle area min", MIN_AREA);
				PAR_LIMIT = (int) SmartDashboard.getNumber("Particle Limit", PAR_LIMIT);
				
				if (numParticles > 0) {					
					ArrayList<Particle> qualifyingParticles = new ArrayList<Particle>();
					for (int particleIndex = 0; particleIndex < numParticles; particleIndex++) {
						Particle par = new Particle(NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP),
													NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT),
													NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM),
													NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT));
						double temp = par.getAspect();
						if (par.getArea() > MIN_AREA && temp < RATIO_MAX && temp > RATIO_MIN) {
							qualifyingParticles.add(par);
							//if (qualifyingParticles.size() > particleLimit) qualifyingParticles.remove(qualifyingParticles.size() - 1);
						}
					}
					SmartDashboard.putNumber("Filtered particles", qualifyingParticles.size());
					
					// Set driving limits
					min_speed = SmartDashboard.getNumber("Min Speed");
					max_speed = SmartDashboard.getNumber("Max Speed");
					
					// If it found a particle, point towards it, then drive towards it
					if(qualifyingParticles.size() > 0) {
						// Finds the best particle
						Particle bestPar = qualifyingParticles.get(0);
						for (int i = 1; i < qualifyingParticles.size(); i++)
							if (Math.abs(RATIO - qualifyingParticles.get(i).getAspect()) < Math.abs(RATIO - bestPar.getAspect()))
								bestPar = qualifyingParticles.get(i);
						// Saves the direction
						direction = bestPar.getDirection();
						// Points at goal
						if (SmartDashboard.getBoolean("do Aim?", false)) turn(direction, bestPar.getDistance());
						else setMotors(0,0);
						// Displays information
						SmartDashboard.putNumber("Area", bestPar.getArea());
						SmartDashboard.putNumber("Left", bestPar.getLeft()/640.0);
						SmartDashboard.putNumber("Right", bestPar.getRight()/640.0);
						SmartDashboard.putNumber("Top", bestPar.getTop()/480.0);
						SmartDashboard.putNumber("Bottom", bestPar.getBottom()/480.0);
						SmartDashboard.putNumber("Aspect", bestPar.getAspect());
						SmartDashboard.putNumber("Distance",  bestPar.getDistance());
						SmartDashboard.putNumber("Direction", direction);
						drawRectangle(binaryFrame, bestPar);
						drawRectangle(frame, bestPar);
					}
					else {
						// Turns in the last known direction, doesn't move forward or back if the last known direction
						// was close to 0.5
						if (SmartDashboard.getBoolean("do Aim?", false)) turn(direction, 200);
						else setMotors(0,0);
					}
					
				}
				
				// Send masked image to dashboard to assist in tweaking mask.
				if(SmartDashboard.getBoolean("binaryFrame?", false)) CameraServer.getInstance().setImage(binaryFrame);
				else CameraServer.getInstance().setImage(frame);
			}
		}
	}
	
	private void turn(double dir, double dist) {
		// linear function
		if (Math.abs(dir - 0.5) < 0.2) {
			setMotors((dist-200)/10.0, (dist-200)/10.0); // try to maintain a distance of 200 inches
		} else {
			setMotors(limit((dir - 0.5) / 2.0), limit((dir - 0.5) / 2.0)); // Keeps speed within max and min speeds
		}
		// sign function:
		//if (dir < .4) setMotors(-0.13, -0.13);
		//else if (dir > .6) setMotors(0.13, 0.13);
		//else setMotors(0,0);
	}
	
	private void setMotors(double l, double r) {
		left1.set(l); left2.set(l); left3.set(-l); 
		right1.set(r); right2.set(r); right3.set(-r);
	}
	
	private double limit(double speed) {
		return (speed > 0 ? 1 : -1) * (Math.abs(speed) < min_speed ? min_speed : (Math.abs(speed) > max_speed ? max_speed : Math.abs(speed)));
		// What that means (not exactly but equivalent result):
		/* if (speed > 0) {
		 *    if (Math.abs(speed) < min_speed) return min_speed;
		 *    else {
		 *        if (Math.abs(speed)) > max_speed) return max_speed;
		 *        return -1*Math.abs(speed);
		 *    }
		 * } else {
		 * 	  if (Math.abs(speed) < min_speed) return -1*min_speed;
		 *    else {
		 *        if (Math.abs(speed)) > max_speed) return -1*max_speed;
		 *        else return -1*Math.abs(speed);
		 *    }
		 * }
		 */
	}
	
	private void drawRectangle(Image image, Particle par) {
		NIVision.Rect rect = new NIVision.Rect((int) par.getTop(),
											   (int) par.getLeft(),
											   (int) (par.getBottom() - par.getTop()),
											   (int)(par.getRight() - par.getLeft()));
		NIVision.imaqDrawShapeOnImage(image,
									  image,
									  rect,
									  NIVision.DrawMode.DRAW_VALUE,
									  NIVision.ShapeMode.SHAPE_RECT, (float) 5.0); 
		// Not sure what the draw mode, shape mode, and newPixelValue (the last 3 parameters) are fore
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