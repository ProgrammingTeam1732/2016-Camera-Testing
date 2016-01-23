package org.usfirst.frc.team1732.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the CameraServer class.
 * With start automatic capture, there is no opportunity to process the image.
 * Look at the IntermediateVision sample for how to process the image before sending it to the FRC PC Dashboard.
 */
public class Robot extends SampleRobot {

	DigitalInput[] DIO;
	//CameraServer server;
	Encoder encoder;
	Joystick controller;
	Image bob;
	int billy;

    public Robot() {
		controller = new Joystick(0);
    	DIO = new DigitalInput[7];
    	for(int i = 0; i < 7 ; i++) {
    		DIO[i] = new DigitalInput(i);
    	}
    	encoder = new Encoder(8, 9);
    	//server = CameraServer.getInstance();
        //server.setQuality(50);
        //server.startAutomaticCapture("cam0");
        bob = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
        billy = NIVision.IMAQdxOpenCamera("cam0", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
        NIVision.IMAQdxConfigureGrab(billy);
        SmartDashboard.putBoolean("Bla", false);
    }
    
    public void dioToDashboard(int port) {
    	SmartDashboard.putBoolean("DIO " + port, DIO[port].get());
    }
    
    public void sendImage(boolean theKid) {
    	if(theKid) {
    		NIVision.IMAQdxGrab(billy, bob, 1);
    		CameraServer.getInstance().setImage(bob);
    	}
    }

    public void operatorControl() {
    	NIVision.IMAQdxStartAcquisition(billy);
    	encoder.reset();
        while (isOperatorControl() && isEnabled()) {
        	sendImage(SmartDashboard.getBoolean("Bla", false));
        	//sendImage(controller.getRawButton(0));
        	for(int i = 0; i < 7; i++) dioToDashboard(i);
        	SmartDashboard.putNumber("Raw", encoder.getRaw());
            Timer.delay(0.005);		// wait for a motor update time
        }
        NIVision.IMAQdxStopAcquisition(billy);
    }
    
}
