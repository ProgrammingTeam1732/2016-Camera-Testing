package org.usfirst.frc.team1732.robot;

import java.util.ArrayList;
import java.util.Comparator;

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

	//* A structure to hold measurements of a particle
	public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport> {
		double PercentAreaToImageArea;
		double Area;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;

		public int compareTo(ParticleReport r) {return (int) (r.Area - this.Area);}
		public int compare(ParticleReport r1, ParticleReport r2) {return (int) (r1.Area - r2.Area);}
	};//*/
	
	public class Scores {
		double Area;
		double Aspect;
	};

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
	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
	NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(0, 0, 1, 1);
	Scores scores = new Scores();

	public Robot() {
		
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_U8, 0);

		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM, 100.0, 0, 0);
		//criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_RATIO_OF_EQUIVALENT_ELLIPSE_AXES, RATIO_MIN, RATIO_MAX, 0, 0);
				
		session = NIVision.IMAQdxOpenCamera("cam0", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		NIVision.IMAQdxConfigureGrab(session);
		SmartDashboard.putBoolean("Capture?", false);
		SmartDashboard.putBoolean("binaryFrame?", false);
		SmartDashboard.putBoolean("Filter by area?", false);
		// Put default values to SmartDashboard so fields will appear
		SmartDashboard.putNumber("Tote hue min", GOAL_HUE_RANGE.minValue);
		SmartDashboard.putNumber("Tote hue max", GOAL_HUE_RANGE.maxValue);
		SmartDashboard.putNumber("Tote sat min", GOAL_SAT_RANGE.minValue);
		SmartDashboard.putNumber("Tote sat max", GOAL_SAT_RANGE.maxValue);
		SmartDashboard.putNumber("Tote val min", GOAL_VAL_RANGE.minValue);
		SmartDashboard.putNumber("Tote val max", GOAL_VAL_RANGE.maxValue);
		SmartDashboard.putNumber("Area min %", AREA_MINIMUM);
		//SmartDashboard.putNumber("Ratio min", RATIO_MIN);
		//SmartDashboard.putNumber("Ratio max", RATIO_MAX);
	}

	public void operatorControl() {
		NIVision.IMAQdxStartAcquisition(session);
		// encoder.reset();
		while (isOperatorControl() && isEnabled()) {
			// SmartDashboard.putBoolean("Photosensor", DIO[0]()); // For the photosensor, if we need more just use for loop
			// SmartDashboard.putNumber("Encoder Raw", encoder.getRaw()); // For the encoder
			if (SmartDashboard.getBoolean("Capture?", false)) {
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

				// filter out small particle
				// delete next two lines after finishing testing
				/*float ratioMin = (float) SmartDashboard.getNumber("Ratio min", RATIO_MIN);
				float ratioMax = (float) SmartDashboard.getNumber("Ratio max", RATIO_MAX);
				criteria[0].lower = ratioMin;
				criteria[0].upper = ratioMax;
				imaqError = NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);*/
				if (SmartDashboard.getBoolean("Filter by area?", false)) {
					float areaMin = (float) SmartDashboard.getNumber("Area min %", AREA_MINIMUM);
					criteria[0].lower = areaMin;
					numberParticles = NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);
				}
				
				// Send particle count after filtering to dashboard
				numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
				SmartDashboard.putNumber("Filtered particles", numParticles);
				SmartDashboard.putNumber("Number of particles from Filter Method", numberParticles);

				if (numParticles > 0) {					
					// Measure particles and sort by particle size
					ArrayList<ParticleReport> particles = new ArrayList<ParticleReport>();
					for (int particleIndex = 0; particleIndex < numParticles; particleIndex++) {
						ParticleReport par = new ParticleReport();
						par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
						par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
						par.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
						par.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
						par.BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
						par.BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
						particles.add(par);
					}
					// How does this sort the particles? We want to be sure it is actually finding the biggest one.
					// Also even though it is very likely the biggest particle will always be the goal, it might be better to
					// sort by something that would be specific to the goal. Also since we know the number of particles
					// beforehand couldn't this be an array to make it faster, I know the array class has a method to sort things
					particles.sort(null);

					// This example only scores the largest particle. Extending to score all particles and choosing the
					// desired one is left as an exercise for the reader. Note that this scores and reports information
					// about a single particle (single L shaped target). To get accurate information about the location
					// of the tote (not just the distance) you will need to correlate two adjacent targets in order to
					// find the true center of the tote.
					scores.Aspect = AspectScore(particles.get(0));
					SmartDashboard.putNumber("Aspect", scores.Aspect);
					scores.Area = AreaScore(particles.get(0));
					SmartDashboard.putNumber("Area", scores.Area);
					boolean isGoal = scores.Aspect > SCORE_MIN && scores.Area > SCORE_MIN;

					// Send distance and tote status to dashboard. The bounding rect, particularly the
					// horizontal center (left - right) may be useful for rotating/driving towards a tote
					SmartDashboard.putBoolean("Is Goal?", isGoal);
					SmartDashboard.putNumber("Distance", computeDistance(binaryFrame, particles.get(0)));
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

	// Comparator function for sorting particles. Returns true if particle 1 is larger
	static boolean CompareParticleSizes(ParticleReport particle1, ParticleReport particle2) {
		// we want descending sort order
		return particle1.PercentAreaToImageArea > particle2.PercentAreaToImageArea;
	}

	/**
	 * Converts a ratio with ideal value of 1 to a score. The resulting function
	 * is piecewise linear going from (0,0) to (1,100) to (2,0) and is 0 for all
	 * inputs outside the range 0-2
	 */
	double ratioToScore(double ratio) {
		// Also why do we even have the score converted like this? Seems like it would be faster just to compare the raw values
		// Could be problem that it doesn't work for inputs larger than 2 because if the goal appears more than twice as long
		// than high then it will always just return 0 for the aspect score we should think about writing some of our own methods
		// for scoring the particle.
		// Also It seems like finding min is pointless because it will never be more than 100 for all values of ratio,
		// so finding min of the two could be deleted
		return (Math.max(0, Math.min(100 * (1 - Math.abs(1 - ratio)), 100)));
	}

	double AreaScore(ParticleReport report) {
		double boundingArea = (report.BoundingRectBottom - report.BoundingRectTop) * (report.BoundingRectRight - report.BoundingRectLeft);
		// Tape is 7" edge so 49" bounding rect. With 2" wide tape it covers 24" of the rect.
		return ratioToScore((49 / 24) * report.Area / boundingArea);
		// Possibly need to change above values of 49 and 24
		// also the ratioToScore is a constant so it might be better to calculate that and have the constant in there
		// that way the robot does less calculations
	}

	/**
	 * Method to score if the aspect ratio of the particle appears to match the
	 * retro-reflective target. Target is 7"x7" so aspect should be 1
	 */
	double AspectScore(ParticleReport report) {
		return ratioToScore(((report.BoundingRectRight - report.BoundingRectLeft) / (report.BoundingRectBottom - report.BoundingRectTop)));
	}

	/**
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
	 */
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
	}

}
