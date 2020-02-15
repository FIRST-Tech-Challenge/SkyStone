package org.firstinspires.ftc.teamcode.Skystone.Odometry;

import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class Odometry {

    private double oldLeftPod = 0;
    private double oldRightPod = 0;
    private double oldMecanumPod = 0;

    double worldX;
    double worldY;
    double worldAngle;

    private double moveScaleFactor = (0.004177098/125) * 123;
    private double turnScaleFactor = 0.000301686*240;
    private double strafePredictionFactor = -0.25;

    public Odometry() {}

    public void runOdometry(Robot robot) {
        double leftPodNew = -1 * robot.getfLeft().getCurrentPosition(); // fix this for new odo config
        double rightPodNew = -1 * robot.getfRight().getCurrentPosition(); //fix this for new odo config
        double mecanumPodNew = -1 * robot.getbLeft().getCurrentPosition(); // fix this for new odo config

        double dLeftPod = leftPodNew - oldLeftPod;
        double dRightPod = rightPodNew - oldRightPod;
        double dMecanumPod = mecanumPodNew - oldMecanumPod;

        oldLeftPod = leftPodNew;
        oldRightPod = rightPodNew;
        oldMecanumPod = mecanumPodNew;

        smallAngleOdometry(dLeftPod, dRightPod, dMecanumPod);
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

    public void resetOdometry(){
        oldLeftPod = 0;
        oldRightPod = 0;
        oldMecanumPod = 0;

        worldX = 0;
        worldY = 0;
        worldAngle = 0;
    }
}