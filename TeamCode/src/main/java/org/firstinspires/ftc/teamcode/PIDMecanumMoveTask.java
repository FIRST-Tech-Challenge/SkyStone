package org.firstinspires.ftc.teamcode;

/**
 * 2019.10.26
 * PID
 * Created by Claire Zheng
 */

public class PIDMecanumMoveTask implements RobotControl {

    RobotPosition startPos, endPos;

    transient RobotHardware robot;
    transient RobotProfile profile;
    transient RobotNavigator navigator;
    transient PIDController pidPosition;
    transient PIDController pidHeading;
    transient boolean completed;
    transient int loopCount = 0;
    //transient double pathAngle = 0;
    transient double pathDistance = 0;
    transient double targetAngle;

    public PIDMecanumMoveTask(RobotHardware robot, RobotProfile profile,RobotNavigator  navigator){
        this.robot = robot;
        this.profile = profile;
        this.navigator = navigator;
        startPos = null;
        endPos = null;
        completed = false;

    }

    public void setPath(RobotPosition startPos, RobotPosition endPos) {
        this.startPos = startPos;
        this.endPos = endPos;
        Logger.logFile("Path Task:" + startPos.getX() + "," + startPos.getY() + " TO " +
                endPos.getX() + "," + endPos.getY());
        //pathAngle = Math.PI / 2 - Math.atan2(endPos.y - startPos.y, endPos.x - startPos.x);  redundant with targetAngle
        if(profile != null){
            setupPID();
        }
    }

    void setupPID(){
        pidPosition = new PIDController(profile.distancePID.p, profile.headingPID.i, profile.headingPID.d);
        pidHeading = new PIDController(profile.headingPID.p, profile.headingPID.i, profile.headingPID.d);
        pidPosition.reset();
        pidPosition.setSetpoint(0);
        pidPosition.setInputRange(-5, 5);
        pidPosition.setOutputRange(0, Math.PI/10);
        pidHeading.reset();
        pidHeading.setInputRange(-Math.PI / 10, Math.PI / 10);
        pidHeading.setOutputRange(0, .2);
        pidHeading.setSetpoint(endPos.getHeading());
    }

    public double getPosError() {
        double currentAngle = Math.PI/2 - Math.atan2(navigator.getWorldY() - startPos.getY(), navigator.getWorldX() - startPos.getX());
        double error = Math.hypot(navigator.getWorldY() - startPos.getY(), navigator.getWorldX() - startPos.getX()) * Math.sin(-targetAngle + currentAngle);
        return error;
    }

    public double getHeadingError() {
        return navigator.getHeading() - endPos.getHeading();
    }

    public boolean isDone(){
        double targetDistance = Math.hypot(endPos.getY() - startPos.getY(), endPos.getX() - startPos.getX());
        double currentDistance = Math.hypot(navigator.getWorldY() - startPos.getY(), navigator.getWorldX() - startPos.getX());
        Logger.logFile("currentDistance: " + currentDistance + ", targetDistance: " + targetDistance);
        return currentDistance > targetDistance;
    }

    public void prepare(){
        pidHeading.enable();
        pidPosition.enable();
        robot.setMotorStopBrake(true);
        targetAngle = Math.PI/2-Math.atan2(endPos.getY() - startPos.getY(), endPos.getX() - startPos.getX());
    }

    public void execute() {
        double posCorrection = pidPosition.performPID(getPosError());
        double headCorrection = pidPosition.performPID(getHeadingError());
        double targetDistance = Math.hypot(endPos.getY() - startPos.getY(), endPos.getX() - startPos.getX());
        double currentDistance = Math.hypot(navigator.getWorldY() - startPos.getY(), navigator.getWorldX() - startPos.getX());
        if (targetDistance - currentDistance <= profile.movement.forwardStopDist){
            double pwr = 0.2 + (targetDistance - currentDistance)/profile.movement.forwardStopDist * 0.3;
            robot.mecanumDrive2(pwr, targetAngle + posCorrection, headCorrection);
        }
        else{
            robot.mecanumDrive2(0.5,targetAngle + posCorrection, headCorrection);
        }
        Logger.logFile("X:" + navigator.getWorldX() + ", Y:" + navigator.getWorldY() +
                 " H:" + navigator.getHeading() + ", PosCorr:" + posCorrection + " ,headCorr:" + headCorrection);
    }
    public void cleanUp(){
        robot.setMotorPower(0,0,0,0);
    }

}
