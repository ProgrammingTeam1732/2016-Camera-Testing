package org.usfirst.frc.team1732.robot;

import java.util.ArrayList;
import java.util.Collections;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
	
public class Robot extends SampleRobot {
	
	DigitalInput[] DIO;
	Encoder encoder;
	Joystick controller;
	int session;

	// Images
	Image frame;
	Image binaryFrame;
	int numberParticles;

	// Constants
	NIVision.Range GOAL_HUE_RANGE = new NIVision.Range(67, 82); // Default hue range for goal
	NIVision.Range GOAL_SAT_RANGE = new NIVision.Range(95, 110); // Default saturation range for goal
	NIVision.Range GOAL_VAL_RANGE = new NIVision.Range(243, 255); // Default value range for goal
	
	double AREA_MINIMUM = 0.5; // Default Area minimum for particle as a percentage of total image area
	double RATIO = 1.428571; // Goal width = 20 in. / goal height = 12 in. = 1.428
	double RATIO_MIN = 1.328571; // Goal width = 20 in. / goal height = 12 in. = 1.428
	double RATIO_MAX = 1.528571; // Goal width = 20 in. / goal height = 12 in. = 1.428
	double SCORE_MIN = 75.0; // Minimum score to be considered a goal
	double VIEW_ANGLE = 77.8; // View angle for camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 52 for HD3000 square, 60 for HD3000 640x480

	public Robot() {
		
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_U8, 0);
				
		session = NIVision.IMAQdxOpenCamera("cam0", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		NIVision.IMAQdxConfigureGrab(session);
		SmartDashboard.putBoolean("Capture?", false);
		SmartDashboard.putBoolean("binaryFrame?", false);
		SmartDashboard.putBoolean("Filter by area?", false);

		SmartDashboard.putNumber("Tote hue min", GOAL_HUE_RANGE.minValue);
		SmartDashboard.putNumber("Tote hue max", GOAL_HUE_RANGE.maxValue);
		SmartDashboard.putNumber("Tote sat min", GOAL_SAT_RANGE.minValue);
		SmartDashboard.putNumber("Tote sat max", GOAL_SAT_RANGE.maxValue);
		SmartDashboard.putNumber("Tote val min", GOAL_VAL_RANGE.minValue);
		SmartDashboard.putNumber("Tote val max", GOAL_VAL_RANGE.maxValue);
		SmartDashboard.putNumber("Area min %", AREA_MINIMUM);
	}

	public void operatorControl() {
		NIVision.IMAQdxStartAcquisition(session);
		
		while (isOperatorControl() && isEnabled()) {
			
			if (SmartDashboard.getBoolean("Capture?", false)) {
				
				// get the image and 
				NIVision.IMAQdxGrab(session, frame, 1);

				// Update threshold values from SmartDashboard. For performance reasons it is recommended to remove
				// this after calibration is finished.
				GOAL_HUE_RANGE.minValue = (int) SmartDashboard.getNumber("Tote hue min", GOAL_HUE_RANGE.minValue);
				GOAL_HUE_RANGE.maxValue = (int) SmartDashboard.getNumber("Tote hue max", GOAL_HUE_RANGE.maxValue);
				GOAL_SAT_RANGE.minValue = (int) SmartDashboard.getNumber("Tote sat min", GOAL_SAT_RANGE.minValue);
				GOAL_SAT_RANGE.maxValue = (int) SmartDashboard.getNumber("Tote sat max", GOAL_SAT_RANGE.maxValue);
				GOAL_VAL_RANGE.minValue = (int) SmartDashboard.getNumber("Tote val min", GOAL_VAL_RANGE.minValue);
				GOAL_VAL_RANGE.maxValue = (int) SmartDashboard.getNumber("Tote val max", GOAL_VAL_RANGE.maxValue);

				// Threshold the image looking for yellow (tote color)
				NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV, GOAL_HUE_RANGE, GOAL_SAT_RANGE, GOAL_VAL_RANGE);

				// Send particle count to dashboard
				int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
				SmartDashboard.putNumber("Masked particles", numParticles);
				
				// Send particle count after filtering to dashboard
				numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
				SmartDashboard.putNumber("Filtered particles", numParticles);
				SmartDashboard.putNumber("Number of particles from Filter Method", numberParticles);

				if (numParticles > 0) {					
					// Measure particles and sort by particle size
					ArrayList<Particle> particles = new ArrayList<Particle>();
					for (int particleIndex = 0; particleIndex < numParticles; particleIndex++) {
						Particle par = new Particle(	NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP),
														NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT),
														NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM),
														NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT));
						particles.add(par);
					}
					
					Collections.sort(particles, Particle.ParticleComparator);
					
					SmartDashboard.putNumber("Left", particles.get(0).getLeft());
					SmartDashboard.putNumber("Right", particles.get(0).getRight());
					SmartDashboard.putNumber("Top", particles.get(0).getTop());
					SmartDashboard.putNumber("Bottom", particles.get(0).getBottom());
					
					// particles.sort(null);
				} else {
					SmartDashboard.putBoolean("IsTote", false);
				}
				
				// Send masked image to dashboard to assist in tweaking mask.
				if(SmartDashboard.getBoolean("binaryFrame?", false)) CameraServer.getInstance().setImage(binaryFrame);
				else CameraServer.getInstance().setImage(frame);
			}
		}
		NIVision.IMAQdxStopAcquisition(session);
	}

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
