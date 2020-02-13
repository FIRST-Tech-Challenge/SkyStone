package org.firstinspires.ftc.teamcode.Skystone.Odometry;

import org.firstinspires.ftc.teamcode.Skystone.Robot;


// The letter "d" preceding variable names in this class indicates delta
public class Odometry {
    //for constant velo

    public double oldLeftPod = 0;
    public double oldRightPod = 0;
    public double oldMecanumPod = 0;

    double xPosGlobal = 0;
    double yPosGlobal = 0;
    double angleGlobal = 0;

    private double fLeftOLD = 0;
    private double fRightOLD = 0;
    private double bLeftOLD = 0;
    private double bRightOLD = 0;

    //circular odometry stuff
    double mecanumPodOld;
    double rightPodOld;
    double leftPodOld;

    double worldX;
    double worldY;
    double worldAngle;

    double moveScaleFactor = (0.004177098/125) * 123;
    double turnScaleFactor = 0.000301686*240;
    double strafePredictionFactor = 0.092;

//    double encoderToInches = ;

    public Odometry() {

    }

    public void circularOdometry (Robot robot) {
        double leftPodNew = -1 * robot.getfLeft().getCurrentPosition(); // fix this for new odo config
        double rightPodNew = -1 * robot.getfRight().getCurrentPosition(); //fix this for new odo config
        double mecanumPodNew = -1 * robot.getbLeft().getCurrentPosition(); // fix this for new odo config

        double dLeftPod = leftPodNew - oldLeftPod;
        double dRightPod = rightPodNew - oldRightPod;
        double dMecanumPod = mecanumPodNew - oldMecanumPod;

        oldLeftPod = leftPodNew;
        oldRightPod = rightPodNew;
        oldMecanumPod = mecanumPodNew;

        circularOdometry(dLeftPod, dRightPod, dMecanumPod);
    }

    /* Circular Odometry assumes that the movement that occurred between each update was in the form
       of an arc (except when the robot moves directly forward. Each time the math is run, the
       algorithm uses coordinate system (x',y') where x' is the direct forward direction between the
       last point and the new point. */
    public void circularOdometry (double dLeftPod, double dRightPod, double dMecanumPod) {
        double dLeftPodInches = dLeftPod * moveScaleFactor;
        double dRightPodInches = dRightPod * moveScaleFactor;
        double dMecanumPodInches = dMecanumPod * moveScaleFactor;

        double dTheta = (dLeftPodInches - dRightPodInches) * turnScaleFactor;

        // Default changes in x' and y' directions assume no change in robot's angle
        double dYPrime = dMecanumPodInches;
        double dXPrime = dRightPodInches;

        // Calculate midAngle, the angle used to convert from x'y' coordinate system to global (x,y) system
        double midAngle = worldAngle + dTheta * .5;

        // Update the global angle of the robot
//        worldAngle += dTheta;
        worldAngle = (((dLeftPod+oldLeftPod) * moveScaleFactor) - ((dRightPod + oldRightPod) * moveScaleFactor)) * turnScaleFactor;

        if (dTheta != 0.0){ // if robot turned
            // Calculate the trigonometry portion of the positions
            double curveFactor = circularOdometrySinXOverX(dTheta / 2);

            dXPrime = (dLeftPodInches + dRightPodInches) * .5 * curveFactor;
            dYPrime = dMecanumPodInches * curveFactor + .25 * 2 * Math.sin(dTheta / 2);
        }

        // Update world x and y positions of the robot by converting from robot's x'y' coordinate
        // system to the global xy coordinate system
        worldX += dXPrime * Math.cos(midAngle) - dYPrime * Math.sin(midAngle);
        worldY += dXPrime * Math.sin(midAngle) + dYPrime * Math.cos(midAngle);
    }

    public void smallAngleOdometry (double dLeftPod, double dRightPod, double dMecanumPod) {
        double dLeftPodInches = dLeftPod * moveScaleFactor;
        double dRightPodInches = dRightPod * moveScaleFactor;
        double dMecanumPodInches = dMecanumPod * moveScaleFactor;

        worldAngle = (((dLeftPod+oldLeftPod) * moveScaleFactor) - ((dRightPod + oldRightPod) * moveScaleFactor)) * turnScaleFactor;

        double dAngle = (dLeftPodInches - dRightPodInches) * turnScaleFactor;
        double dRobotX = (dLeftPodInches + dRightPodInches) * .5;
        double dRobotY = dMecanumPodInches + strafePredictionFactor * dAngle;


        // Update world x and y positions of the robot by converting from robot's x'y' coordinate
        // system to the global xy coordinate system
        worldX += dRobotX * Math.cos(worldAngle + dAngle * 0.5) - dRobotY * Math.sin(worldAngle + dAngle * 0.5);
        worldY += dRobotX * Math.sin(worldAngle + dAngle * 0.5) + dRobotY * Math.cos(worldAngle + dAngle * 0.5);
    }

    private double circularOdometrySinXOverX(double x) {
        if (Math.abs(x) < .00005) { // If the ratio is close enough to the limit, make it the limit
            return 1;
        } else {
            return Math.sin(x) / x;
        }
    }

    public void linearOdometry (Robot robot) {
        double leftPodNew = -1 * robot.getfLeft().getCurrentPosition(); // fix this for new odo config
        double rightPodNew = -1 * robot.getfRight().getCurrentPosition(); //fix this for new odo config
        double mecanumPodNew = -1 * robot.getbLeft().getCurrentPosition(); // fix this for new odo config

        double dLeftPod = leftPodNew - oldLeftPod;
        double dRightPod = rightPodNew - oldRightPod;
        double dMecanumPod = mecanumPodNew - oldMecanumPod;

        oldLeftPod = leftPodNew;
        oldRightPod = rightPodNew;
        oldMecanumPod = mecanumPodNew;

        linearOdometry(dLeftPod, dRightPod, dMecanumPod);
    }

    public void linearOdometry (double dLeftPod, double dRightPod, double dMecanumPod) {
        double dTheta = (dLeftPod - dRightPod) * turnScaleFactor;

        double midAngle = worldAngle + dTheta * .5;

        worldAngle += dTheta;

        double dXPrime = (dLeftPod + dRightPod) * .5 * moveScaleFactor;
        double dYPrime = dMecanumPod * moveScaleFactor;

        worldX += dXPrime * Math.cos(midAngle) + dYPrime * Math.sin(midAngle);
        worldY += -1 * dXPrime * Math.sin(midAngle) + dYPrime * Math.cos(midAngle);
    }

    public void resetOdometry(){
        xPosGlobal = 0;
        yPosGlobal = 0;
        angleGlobal = 0;

        //circular odometry stuff
        mecanumPodOld = 0;
        rightPodOld = 0;
        leftPodOld = 0;

        worldX = 0;
        worldY = 0;
        worldAngle = 0;
    }

    public double getWorldX() {
        return worldX;
    }

    public double getWorldY() {
        return worldY;
    }

    public double getWorldAngle() {
        return worldAngle;
    }
}