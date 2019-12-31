package org.firstinspires.ftc.teamcode.PreseasonTest.MecanumLocalizers;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.openftc.revextensions2.RevBulkData;


// Motivated by http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
public class ICCLocalizer implements BulkReadConsumer{

    /* MA3Encoder leftTrackingWheel = new MA3Encoder(0);
    MA3Encoder rightTrackingWheel = new MA3Encoder(1);
    MA3Encoder horizontalTrackingWheel = new MA3Encoder(2); */

	final double trackWidth = 45.72; // cm
	
	private double xCoordinate;
	private double yCoordinate;
	private double heading;
	
    /* public void init(BulkReadManager bulkReadManager) {
        bulkReadManager.registerHighPriority(leftTrackingWheel);
        bulkReadManager.registerHighPriority(rightTrackingWheel);
        bulkReadManager.registerHighPriority(horizontalTrackingWheel);
    } */

    @Override
    public void bulkReadUpdate(RevBulkData revBulkData) {
        
    }
    
    // Arguments are in cm/s
    // Just test w/ tanklike driving for now??
    private void calculatePosition(double leftWheelVelocity, double rightWheelVelocity, double horizontalWheelVelocity, double deltaTime) {
    	double angularVelocity = (rightWheelVelocity - leftWheelVelocity) / trackWidth; // rad/s?
        double angularDisplacement = angularVelocity * deltaTime;
    	double radiusOfCurvature = 0.5 * trackWidth * (leftWheelVelocity + rightWheelVelocity) / (rightWheelVelocity - leftWheelVelocity);


    	double[][] transformationValues = {{Math.cos(angularDisplacement), -Math.sin(angularDisplacement), 0},
                {Math.sin(angularDisplacement), Math.cos(angularDisplacement), 0},
                {0, 0, 1}};
        RealMatrix transformation = MatrixUtils.createRealMatrix(transformationValues);

        double[] transformedCoordinateValues = {radiusOfCurvature * Math.sin(heading),
                -radiusOfCurvature * Math.sin(heading),
                heading};
        RealVector transformedCoordinates = new ArrayRealVector(transformedCoordinateValues);

        RealVector rotatedCoordinates = transformation.preMultiply(transformedCoordinates);

        double[] coordinateValues = {xCoordinate - radiusOfCurvature * Math.sin(heading),
                yCoordinate - radiusOfCurvature * Math.sin(heading),
                angularDisplacement};

        RealVector coordinates = new ArrayRealVector(coordinateValues);

        RealVector resultant = rotatedCoordinates.add(coordinates);
        xCoordinate = resultant.getEntry(0);
        yCoordinate = resultant.getEntry(1);
        heading = resultant.getEntry(2);
    }
}
