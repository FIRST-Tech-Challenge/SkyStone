package org.firstinspires.ftc.teamcode;

/**
 * 2019.12.01
 * Created by Athena Z.
 * ----
 * Now includes move to new position too.  It won't be done until the distance to target is within 1 cm
 * and the angle is within 1 degree (1/180 * PI)
 */

public class MecanumRotateTask implements RobotControl {

    RobotPosition startAngle, endAngle;
    double power = 0.5;

    transient int rightTurnSign;    //+ for right turn, - for left turn
    transient RobotHardware robot;
    transient RobotProfile profile;
    transient RobotNavigator navigator;
    transient double currentAngle;
    transient double targetAngle;
    transient double minPower = 0.2;

    public MecanumRotateTask(RobotHardware robot, RobotProfile profile, RobotNavigator navigator) {
        this.robot = robot;
        this.profile = profile;
        this.navigator = navigator;
        startAngle = null;
        endAngle = null;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setMinPower(double minPower){
        this.minPower = minPower;
    }

    public void setRotateHeading(RobotPosition startAngle, RobotPosition endAngle) {
        this.startAngle = startAngle;
        this.endAngle = endAngle;
        Logger.logFile("Heading Rotate Task:" + Math.toDegrees(startAngle.getHeading()) + " TO " + Math.toDegrees(endAngle.getHeading()));
    }

    public String toString() {
        return "Rotate from (" + startAngle.getX() + "," + startAngle.getY() + ") @" + Math.toDegrees(startAngle.getHeading()) + " TO (" +
                    endAngle.getX() + "," + endAngle.getY() + ") @" + Math.toDegrees(endAngle.getHeading()) + " curr:" + navigator.getLocationString();
    }

    public boolean isDone() {
        double dist = Math.hypot(endAngle.getY() - navigator.getWorldY(), endAngle.getX() - navigator.getWorldX());
//        if ((dist<1.5) && (Math.abs(navigator.getHeading() - endAngle.getHeading()) < 1.0/180*Math.PI)){
//            Logger.logFile("Done Rotation - Current Heading: " + Math.toDegrees(currentAngle) + ", Target Heading: " + Math.toDegrees(targetAngle));
//            return true;
//        } else {
//            return false;
//        }
//    }
        //12/12 try to loose the condition so hook off easier
        if ((dist < 1.5) && (Math.abs(navigator.getHeading() - endAngle.getHeading()) < 1.0/180*Math.PI)){
            Logger.logFile("Done Rotation - Current Heading: " + Math.toDegrees(currentAngle) + ", Target Heading: " + Math.toDegrees(targetAngle));
            return true;
        } else {
            return false;
        }
    }

    public void prepare() {
        robot.setMotorStopBrake(true);
        currentAngle = startAngle.getHeading();
        targetAngle = endAngle.getHeading();
        rightTurnSign = (endAngle.getHeading()>startAngle.getHeading())? 1 : -1;
        Logger.logFile("Prepare Rotation - Current Heading: " + Math.toDegrees(currentAngle) + ", Target Heading: " + Math.toDegrees(targetAngle));
    }

    public void execute() {
        /**
         * RobotProfile.java    class Movement  double rotateStopAngle;
         * movement.rotateStopAngle = 5;
         */
        currentAngle = navigator.getHeading();
        // calculate the target angle based on current position and end position
        // and move the robot to the target position
        double moveAngle = Math.PI/2-Math.atan2(endAngle.getY() - navigator.getWorldY(), endAngle.getX() - navigator.getWorldX());
        double dist = Math.hypot(endAngle.getY() - navigator.getWorldY(), endAngle.getX() - navigator.getWorldX());
        double movePwr, rotatePwr;
        if (dist>profile.movement.forwardStopDist) {
            movePwr = power;
        }
        else {
            movePwr = minPower + dist/profile.movement.forwardStopDist * (power - minPower);
        }
        // now need to determine which way to turn
        // when it's greater than 45 degree, we use the direction prescribed by start/end angle (rightTurnSign)
        if (Math.abs(currentAngle - endAngle.getHeading())>profile.movement.rotateStopAngle) {
            rotatePwr = rightTurnSign * power;
        }
        else {
            // now direction should be get close to finish, if overshot, turn back
            rotatePwr = minPower + (Math.abs(currentAngle - endAngle.getHeading())-profile.movement.rotateStopAngle) * (power - minPower);
            if (currentAngle>endAngle.getHeading()) {
                // we going to turn left
                rotatePwr = -rotatePwr;
            }
        }
        robot.mecanumDrive2(movePwr, moveAngle - currentAngle, rotatePwr);
//        Logger.logFile("Rotating current (" + navigator.getWorldX() + "," + navigator.getWorldY() + ") @" + Math.toDegrees(navigator.getHeading()) +
//                " move angle:" + Math.toDegrees((moveAngle)) + " Pwr:" + movePwr + " rot:" + rotatePwr);
//        Logger.flushToFile();
    }

    public void cleanUp() {
        robot.setMotorPower(0,0,0,0);
    }
}