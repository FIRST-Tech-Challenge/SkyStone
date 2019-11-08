package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.RobotLogger;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;

public class Robot2DPositionSoftwareTracker extends Robot2DPositionTracker {

    public Robot2DPositionSoftwareTracker(Robot2DPositionIndicator initialPosition) {
        super(initialPosition);
    }

    public Robot2DPositionSoftwareTracker(Robot2DPositionIndicator initialPosition, double RobotWidth, double RobotHeight) {
        super(initialPosition, RobotWidth, RobotHeight);
    }

    public Robot2DPositionSoftwareTracker(Robot2DPositionSoftwareTracker oldTracker) {
        super(oldTracker);
    }


    public void drive_MoveThroughFieldAngle(double angleInDeg, double distance) {
        double angleInRad = Math.toRadians(angleInDeg);
        double xMoved = Math.cos(angleInRad) * distance, yMoved = Math.sin(angleInRad) * distance;
        this.offsetPosition(new Robot2DPositionIndicator(xMoved, yMoved, 0));
    }
    public void drive_MoveThroughRobotAngle(double angleInDeg, double distance) {
        double fieldAng = angleInDeg + 90 + this.getCurrentPosition().getRotationY();
        this.drive_MoveThroughFieldAngle(fieldAng, distance);
    }
    public void drive_MoveThroughRobotAxisOffset(Robot2DPositionIndicator robotAxisValues) {
        Robot2DPositionIndicator tempField = this.fieldAxisFromRobotAxis(robotAxisValues);
        this.setCurrentPosition(tempField);
    }
    public void drive_RotateAroundFieldPoint(Robot2DPositionIndicator fieldPointAndRotation) {
        Robot2DPositionIndicator currentPos = this.getCurrentPosition();
        if (fieldPointAndRotation.getX() == currentPos.getX() && fieldPointAndRotation.getZ() == currentPos.getZ()) {
            this.offsetPosition(new Robot2DPositionIndicator(0, 0, fieldPointAndRotation.getRotationY()));
        } else {
            double[] point = {fieldPointAndRotation.getX(), fieldPointAndRotation.getZ()};
            double[] currentPosArr = {currentPos.getX(), currentPos.getZ()};
            double[] newRobotPosition = XYPlaneCalculations.rotatePointAroundFixedPoint_Deg(point, currentPosArr, fieldPointAndRotation.getRotationY());
            this.setCurrentPosition(new Robot2DPositionIndicator(newRobotPosition[0], newRobotPosition[1], currentPos.getRotationY() + fieldPointAndRotation.getRotationY()));
        }
    }
    public void drive_RotateAroundRobotAxisPoint(Robot2DPositionIndicator robotPointAndRotation) {
        if(robotPointAndRotation.getX() == 0 && robotPointAndRotation.getZ() == 0){
            if(robotPointAndRotation.getRotationY() != 0) {
                this.getCurrentPosition().setRotationY(this.getCurrentPosition().getRotationY() + robotPointAndRotation.getRotationY());
            }
            return;
        }
        Robot2DPositionIndicator tempFieldPointAndRotation = this.fieldAxisFromRobotAxis(robotPointAndRotation);
        tempFieldPointAndRotation.setRotationY(robotPointAndRotation.getRotationY());
        this.drive_RotateAroundFieldPoint(tempFieldPointAndRotation);
    }
    public void drive_RotateAroundRobotOriginWithRadius(double Radius, double DistanceCounterClockwise){
        if(DistanceCounterClockwise != 0) {
            double moveAngleRad = DistanceCounterClockwise / Radius;
            double moveAngleDeg = Math.toDegrees(moveAngleRad);
            this.getCurrentPosition().setRotationY(this.getCurrentPosition().getRotationY() + moveAngleDeg);
        }
    }
}
