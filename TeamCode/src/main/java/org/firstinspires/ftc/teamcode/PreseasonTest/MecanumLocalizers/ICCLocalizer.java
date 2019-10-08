package org.firstinspires.ftc.teamcode.PreseasonTest.MecanumLocalizers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Util;

import org.openftc.revextensions2.RevBulkData;

// Motivated by http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
public class ICCLocalizer implements BulkReadConsumer{

    MA3Encoder leftTrackingWheel = new MA3Encoder(0);
    MA3Encoder rightTrackingWheel = new MA3Encoder(1);
    MA3Encoder horizontalTrackingWheel = new MA3Encoder(2);

	final double trackWidth = 45.72; // cm
	
	private double xCoordinate;
	private double yCoordinate;
	private double angle;
	
    public void init(BulkReadManager bulkReadManager) {
        bulkReadManager.registerHighPriority(leftTrackingWheel);
        bulkReadManager.registerHighPriority(rightTrackingWheel);
        bulkReadManager.registerHighPriority(horizontalTrackingWheel);
    }

    @Override
    public void bulkReadUpdate(RevBulkData revBulkData) {
        
    }
    
    // Arguments are in cm/s
    private void calculatePosition(double leftWheelVelocity, double rightWheelVelocity, double horizontalWheelVelocity, double deltaTime) {
    	double angularVelocity = (rightWheelVelocity - leftWheelVelocity) / trackWidth; // rad/s?
    	double radiusOfCurvature = 0.5 * trackWidth * (leftWheelVelocity + rightWheelVelocity) / (rightWheelVelocity - leftWheelVelocity);
    	
   		double translatedXCoordinate = radiusOfCurvature * Math.sin(angle);
   		double translatedYCoordinate = radiusOfCurvature * -Math.cos(angle);
   		
   		xCoordinate = (xCoordinate - radiusOfCurvature * Math.sin(angle)) + translatedXCoordinate * Math.cos(angularVelocity * deltaTime) - translatedYCoordinate * Math.sin(angularVelocity * deltaTime);
   		yCoordinate = (yCoordinate + radiusOfCurvature * Math.cos(angle)) + translatedXCoordinate * Math.sin(angularVelocity * deltaTime) + translatedYCoordinate * Math.cos(angularVelocity * deltaTime);
   		angle = angle + angularVelocity * deltaTime;
   			
    }
}
