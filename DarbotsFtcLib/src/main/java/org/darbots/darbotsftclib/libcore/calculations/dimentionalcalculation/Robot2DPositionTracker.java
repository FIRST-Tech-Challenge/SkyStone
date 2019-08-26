package org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation;


import android.provider.Settings;

import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;

import java.lang.reflect.Field;

public class Robot2DPositionTracker {

    private Robot2DPositionIndicator m_2DPos;
    private Robot2DPositionIndicator[] m_ExtremePoints;
    private Robot2DPositionIndicator m_FieldMinPoint, m_FieldMaxPoint;
    private boolean m_NeedPositionFix = false;

    public Robot2DPositionTracker(Robot2DPositionIndicator initialPosition, Robot2DPositionIndicator[] robotExtremePoints, Robot2DPositionIndicator fieldMin, Robot2DPositionIndicator fieldMax) {
        this.m_2DPos = initialPosition;
        this.m_FieldMinPoint = fieldMin;
        this.m_FieldMaxPoint = fieldMax;
        this.m_NeedPositionFix = false;
        this.setExtremePoints(robotExtremePoints);
    }

    public Robot2DPositionTracker(Robot2DPositionTracker Tracker){
        this.m_2DPos = new Robot2DPositionIndicator(Tracker.m_2DPos);
        this.m_FieldMinPoint = Tracker.m_FieldMinPoint;
        this.m_FieldMaxPoint = Tracker.m_FieldMaxPoint;
        this.m_NeedPositionFix = Tracker.m_NeedPositionFix;
        this.setExtremePoints(Tracker.m_ExtremePoints);
    }

    protected void fixBouncingBox() {
        boolean needFix = false;
        double[][] RobotExtremes = new double[4][2];
        Robot2DPositionIndicator[] robotExtremes_Indicators = this.getRobotExtremePoints_fieldAxis();
        for (int i = 0; i < 4; i++) {
            RobotExtremes[i][0] = robotExtremes_Indicators[i].getX();
            RobotExtremes[i][1] = robotExtremes_Indicators[i].getZ();
        }
        for (int i = 0; i < 4; i++) {
            double[] EachExtreme = RobotExtremes[i];
            double Offset = 0;
            if (EachExtreme[0] < this.getFieldMinPoint().getX() || EachExtreme[0] > this.getFieldMaxPoint().getX() || EachExtreme[1] < this.getFieldMinPoint().getZ() || EachExtreme[1] > this.getFieldMaxPoint().getZ()) {
                this.m_NeedPositionFix = true;
            }
            if (EachExtreme[0] < this.getFieldMinPoint().getX()) {
                Offset = this.getFieldMinPoint().getX() - EachExtreme[0];
                this.offsetPosition(new Robot2DPositionIndicator(Offset, 0, 0));
                needFix = true;
                break;
            } else if (EachExtreme[0] > this.getFieldMaxPoint().getX()) {
                Offset = EachExtreme[0] - this.getFieldMaxPoint().getX();
                this.offsetPosition(new Robot2DPositionIndicator(-Offset, 0, 0));
                needFix = true;
                break;
            } else if (EachExtreme[1] < this.getFieldMinPoint().getZ()) {
                Offset = this.getFieldMinPoint().getZ() - EachExtreme[1];
                this.offsetPosition(new Robot2DPositionIndicator(0, Offset, 0));
            } else if (EachExtreme[1] > this.getFieldMaxPoint().getZ()) {
                Offset = EachExtreme[1] - this.getFieldMaxPoint().getZ();
                this.offsetPosition(new Robot2DPositionIndicator(0, -Offset, 0));
            }
        }
        if (needFix) {
            //Because offsetPosition() always calls fixBouncingBox(), we can choose to not call it here
            //this.fixBouncingBox();
        }
    }

    public boolean isNeedPositionFix() {
        return this.m_NeedPositionFix;
    }

    public void finishPositionFix() {
        this.m_NeedPositionFix = false;
    }

    public Robot2DPositionIndicator getFieldMinPoint() {
        return this.m_FieldMinPoint;
    }

    public Robot2DPositionIndicator toRobotAxis(Robot2DPositionIndicator FieldAxis){
        double fieldRelativeX = FieldAxis.getX() - this.getPosition().getX(), fieldRelativeZ = FieldAxis.getZ() - this.getPosition().getZ();
        double[] fieldRelativePoint = {fieldRelativeX, fieldRelativeZ};
        double[] pointOfRotation = {0, 0};
        double rotationAng = -this.getPosition().getRotationY(); //Here the axis plane is rotating into the robot axis plane (going through robot's Y rotation), but the point remain fixed.
        double[] rotatedAxis = XYPlaneCalculations.rotatePointAroundFixedPoint_Deg(fieldRelativePoint, pointOfRotation, rotationAng);
        double rotatedAng = FieldAxis.getRotationY() - this.getPosition().getRotationY();
        return new Robot2DPositionIndicator(rotatedAxis[0], rotatedAxis[1], rotatedAng);
    }

    public Robot2DPositionIndicator toFieldAxis(Robot2DPositionIndicator robotAxisIndicator){
        double[] robotAxisPoint = {robotAxisIndicator.getX(), robotAxisIndicator.getZ()};
        double[] pointOfRotation = {0, 0};
        double rotationAng = Robot2DPositionTracker.this.getPosition().getRotationY();
        double[] rotatedAxis = XYPlaneCalculations.rotatePointAroundFixedPoint_Deg(robotAxisPoint, pointOfRotation, rotationAng);
        double rotatedAng = robotAxisIndicator.getRotationY() + Robot2DPositionTracker.this.getPosition().getRotationY();
        return new Robot2DPositionIndicator(rotatedAxis[0] + this.getPosition().getX(),rotatedAxis[1] + this.getPosition().getZ(),rotatedAng);
    }

    public void setFieldMinPoint(Robot2DPositionIndicator minPoint) {
        this.m_FieldMinPoint = minPoint;
        this.fixBouncingBox();
    }

    public Robot2DPositionIndicator getFieldMaxPoint() {
        return this.m_FieldMaxPoint;
    }

    public void setFieldMaxPoint(Robot2DPositionIndicator maxPoint) {
        this.m_FieldMaxPoint = maxPoint;
        this.fixBouncingBox();
    }

    public Robot2DPositionIndicator[] getRobotExtremePoints() {
        return this.m_ExtremePoints;
    }

    public Robot2DPositionIndicator[] getRobotExtremePoints_fieldAxis() {
        Robot2DPositionIndicator[] result = new Robot2DPositionIndicator[4];
        for (int i = 0; i < 4; i++) {
            result[i] = this.toFieldAxis(this.m_ExtremePoints[i]);
        }
        return result;
    }

    public void setExtremePoints(Robot2DPositionIndicator[] extremes) {
        if (extremes.length != 4) {
            throw new RuntimeException("Length of RobotExtremePoints is not 4");
        }
        this.m_ExtremePoints = extremes;
        this.fixBouncingBox();
    }

    public Robot2DPositionIndicator getPosition() {
        return this.m_2DPos;
    }

    public void setPosition(Robot2DPositionIndicator newPosition) {
        this.m_2DPos = newPosition;
        this.fixBouncingBox();
    }

    public void offsetPosition(Robot2DPositionIndicator offsetPosition) {
        this.m_2DPos.setX(this.m_2DPos.getX() + offsetPosition.getX());
        this.m_2DPos.setZ(this.m_2DPos.getZ() + offsetPosition.getZ());
        this.m_2DPos.setRotationY(this.m_2DPos.getRotationY() + offsetPosition.getRotationY());
        this.fixBouncingBox();
    }

    public void drive_MoveThroughFieldAngle(double angleInDeg, double distance) {
        GlobalRegister.runningOpMode.getRobotCore().getLogger().addLog("Robot2DPositionTracker","driveThroughFieldAngle","Angle=" + angleInDeg + ",Distance=" + distance);
        double angleInRad = Math.toRadians(angleInDeg);
        double xMoved = Math.cos(angleInRad) * distance, yMoved = Math.sin(angleInRad) * distance;
        this.offsetPosition(new Robot2DPositionIndicator(xMoved, yMoved, 0));
        this.__putCurrentPosIntoLog();
    }

    public void drive_MoveThroughRobotAngle(double angleInDeg, double distance) {
        GlobalRegister.runningOpMode.getRobotCore().getLogger().addLog("Robot2DPositionTracker","driveThroughRobotAngle","Angle=" + angleInDeg + ",Distance=" + distance);
        Robot2DPositionIndicator tempRobotIndicator = new Robot2DPositionIndicator(0, 0, angleInDeg);
        Robot2DPositionIndicator tempFieldIndicator = this.toFieldAxis(tempRobotIndicator);
        this.drive_MoveThroughFieldAngle(tempFieldIndicator.getRotationY(), distance);
    }

    public void drive_MoveThroughRobotAxisOffset(Robot2DPositionIndicator robotAxisValues) {
        Robot2DPositionIndicator tempField = this.toFieldAxis(robotAxisValues);
        Robot2DPositionIndicator currentPosition = this.getPosition();
        Robot2DPositionIndicator offsetPosition = new Robot2DPositionIndicator(tempField.getX() - currentPosition.getX(),tempField.getZ() - currentPosition.getZ(),tempField.getRotationY() - currentPosition.getRotationY());
        this.offsetPosition(offsetPosition);
    }

    public void drive_RotateAroundFieldPoint(Robot2DPositionIndicator fieldPointAndRotation) {
        GlobalRegister.runningOpMode.getRobotCore().getLogger().addLog("Robot2DPositionTracker","rotateAroundFieldPoint","X=" + fieldPointAndRotation.getX() + ",Z=" + fieldPointAndRotation.getZ() + ",Angle=" + fieldPointAndRotation.getRotationY());
        if (fieldPointAndRotation.getX() == this.getPosition().getX() && fieldPointAndRotation.getZ() == this.getPosition().getZ()) {
            this.offsetPosition(new Robot2DPositionIndicator(0, 0, fieldPointAndRotation.getRotationY()));
        } else {
            double[] point = {fieldPointAndRotation.getX(), fieldPointAndRotation.getZ()};
            double[] currentPos = {this.getPosition().getX(), this.getPosition().getZ()};
            double[] newRobotPosition = XYPlaneCalculations.rotatePointAroundFixedPoint_Deg(point, currentPos, fieldPointAndRotation.getRotationY());
            this.setPosition(new Robot2DPositionIndicator(newRobotPosition[0], newRobotPosition[1], this.getPosition().getRotationY() + fieldPointAndRotation.getRotationY()));
        }
        this.__putCurrentPosIntoLog();
    }

    public void drive_RotateAroundRobotAxisPoint(Robot2DPositionIndicator robotPointAndRotation) {
        GlobalRegister.runningOpMode.getRobotCore().getLogger().addLog("Robot2DPositionTracker","rotateAroundRobotPoint","X=" + robotPointAndRotation.getX() + ",Z=" + robotPointAndRotation.getZ() + ",Angle=" + robotPointAndRotation.getRotationY());
        Robot2DPositionIndicator tempFieldPointAndRotation = this.toFieldAxis(robotPointAndRotation);
        tempFieldPointAndRotation.setRotationY(robotPointAndRotation.getRotationY());
        this.drive_RotateAroundFieldPoint(tempFieldPointAndRotation);
    }

    public void drive_RotateAroundFieldPointWithRadiusAndPowerPoint(Robot2DPositionIndicator fieldPoint, double Radius, double DistanceCounterClockwise) {
        GlobalRegister.runningOpMode.getRobotCore().getLogger().addLog("Robot2DPositionTracker","rotateAroundFieldPointWithRadiusAndPowerPoint", "FieldX=" + fieldPoint.getX() + ",FieldZ=" + fieldPoint.getZ() + ",Radius=" + Radius + ",DistanceCounterClockwise=" + DistanceCounterClockwise);
        double moveAngleRad = DistanceCounterClockwise / Radius;
        double moveAngleDeg = Math.toDegrees(moveAngleRad);
        this.drive_RotateAroundFieldPoint(new Robot2DPositionIndicator(fieldPoint.getX(), fieldPoint.getZ(), moveAngleDeg));
    }
    public void drive_RotateAroundRobotPointWithRadiusAndPowerPoint(Robot2DPositionIndicator robotAxisPoint, double Radius, double DistanceCounterClockwise){
        GlobalRegister.runningOpMode.getRobotCore().getLogger().addLog("Robot2DPositionTracker","rotateAroundRobotPointWithRadiusAndPowerPoint","RobotX=" + robotAxisPoint.getX() + ",FieldZ=" + robotAxisPoint.getZ() + ",Radius=" + Radius + ",DistanceCounterClockwise=" + DistanceCounterClockwise);
        double moveAngleRad = DistanceCounterClockwise / Radius;
        double moveAngleDeg = Math.toDegrees(moveAngleRad);
        this.drive_RotateAroundRobotAxisPoint(new Robot2DPositionIndicator(robotAxisPoint.getX(), robotAxisPoint.getZ(), moveAngleDeg));
    }
    protected void __putCurrentPosIntoLog(){
        Robot2DPositionIndicator Robot2DPos = this.getPosition();
        GlobalRegister.runningOpMode.getRobotCore().getLogger().addLog("Robot2DPositionTracker","RobotPos","(" + Robot2DPos.getX() + "," + Robot2DPos.getZ() + ")[" + Robot2DPos.getRotationY() + "]");
    }
}