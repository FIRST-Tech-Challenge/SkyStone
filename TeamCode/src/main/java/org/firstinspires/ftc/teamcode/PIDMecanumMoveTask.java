package org.firstinspires.ftc.teamcode;

/**
 * 2019.10.26
 * PID
 * Created by Claire Zheng
 */

public class PIDMecanumMoveTask implements RobotControl {

    RobotPosition startPos, endPos;
    double offsetX;
    double offsetY;
    double power;
    double[] prevDist;  // a ring buffer of previous distance
    int prevNdx;

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
    transient double minPower = 0.2;

    public PIDMecanumMoveTask(RobotHardware robot, RobotProfile profile,RobotNavigator  navigator){
        this.robot = robot;
        this.profile = profile;
        this.navigator = navigator;
        startPos = null;
        endPos = null;
        completed = false;
        power = 0.5;    // default
        prevDist = new double[40];  // previous 10 distance
        for(int i=0; i<prevDist.length; i++) {
            prevDist[i] = -9999;
        }
        prevNdx = 0;
    }

    public String toString() {
        return "Move " + startPos.getX() + "," + startPos.getY() + " -> " + endPos.getX() + "," + endPos.getY() + " curr:" + navigator.getLocationString();
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setMinPower(double minPower){
        this.minPower = minPower;
    }

    public void setPath(RobotPosition startPos, RobotPosition endPos) {
        this.startPos = startPos;
        this.endPos = endPos;
        Logger.logFile("Path Task:" + startPos.getX() + "," + startPos.getY() + " TO " +
                endPos.getX() + "," + endPos.getY());
        //pathAngle = Math.PI / 2 - Math.atan2(endPos.y - startPos.y, endPos.x - startPos.x);  redundant with targetAngle
    }

    public void setRelativePath(double offsetX, double offsetY){
        this.offsetX = offsetX;
        this.offsetY = offsetY;
        startPos = null;
        endPos = null;
    }

    void setupPID(){
        pidPosition = new PIDController(profile.distancePID.p, profile.distancePID.i, profile.distancePID.d);
        pidHeading = new PIDController(profile.headingPID.p, profile.headingPID.i, profile.headingPID.d);
        pidPosition.reset();
        pidPosition.setSetpoint(0);
        pidPosition.setInputRange(-5, 5);       // off by 5 cm max
        pidPosition.setOutputRange(0, Math.PI/20);  // this is the desired angle to move
        pidPosition.enable();
        pidHeading.reset();
        pidHeading.setInputRange(-Math.PI / 30, Math.PI / 30);  // of by 6 degrees max
        pidHeading.setOutputRange(0, .3);
        pidHeading.setSetpoint(0);
        pidHeading.enable();
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
//        Logger.logFile("currentDistance: " + currentDistance + ", targetDistance: " + targetDistance);
        boolean noMovement = false;
        if (Math.abs(currentDistance-prevDist[prevNdx])<1) {
            Logger.logFile("NoMovement - " + currentDistance + " : " + prevDist[prevNdx]);
            noMovement = true;
        }
        prevDist[prevNdx] = currentDistance;
        prevNdx = (prevNdx + 1) % prevDist.length;
//        Logger.logFile("navigator.getWorldY()=" + navigator.getWorldY() + " navigator.getWorldX() " + navigator.getWorldX());
//        Logger.logFile("noMovement = " + noMovement);
        return (currentDistance > targetDistance) || noMovement;
    }

    public void prepare(){
        if(startPos == null && endPos == null){
            startPos = new RobotPosition(navigator.getWorldX(),navigator.getWorldY(),navigator.getHeading());
            endPos = new RobotPosition(offsetX + navigator.getWorldX(), offsetY + navigator.getWorldY(), navigator.getHeading());
        }
        targetAngle = Math.PI/2-Math.atan2(endPos.getY() - startPos.getY(), endPos.getX() - startPos.getX());
        setupPID();
    }

    public void execute() {
        double posCorrection = pidPosition.performPID(getPosError());
        double headCorrection = pidHeading.performPID(getHeadingError());
        double targetDistance = Math.hypot(endPos.getY() - startPos.getY(), endPos.getX() - startPos.getX());
        double currentDistance = Math.hypot(navigator.getWorldY() - startPos.getY(), navigator.getWorldX() - startPos.getX());
        if (targetDistance - currentDistance <= profile.movement.forwardStopDist){
            double pwr = minPower + (targetDistance - currentDistance)/profile.movement.forwardStopDist * (power-minPower);
            robot.mecanumDrive2(pwr, targetAngle + posCorrection - endPos.getHeading(), headCorrection);
        }
        else{
            robot.mecanumDrive2(power,targetAngle + posCorrection - endPos.getHeading(), headCorrection);
        }
        //Logger.logFile("X:" + navigator.getWorldX() + ", Y:" + navigator.getWorldY() +
        //         " H:" + navigator.getHeading() + ", PosCorr:" + posCorrection + " ,headCorr:" + headCorrection);
    }
    public void cleanUp(){
        robot.setMotorPower(0,0,0,0);
    }

}
