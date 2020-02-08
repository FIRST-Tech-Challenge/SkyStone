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
    double turnScaleFactor = (360.0/4927.0);
//    double strafeScaleFactor = 0.004135465;
//    double strafePredictionScalingFactor = 0.092;

//    double encoderToInches = ;

    public Odometry() {

    }


//    double oldLEncoder;
//    double oldREndcoder;
//    double oldSEncoder;
//
//    public void odometry() {
//        double newLEncoder = -1 * robot.getfLeft().getCurrentPosition();
//        double newREncoder = -1 * robot.getfRight().getCurrentPosition();
//        double newSEncoder = -1 * robot.getbLeft().getCurrentPosition();
//
//        double deltaL = (newLEncoder - oldLEncoder) * encoderToInches;
//        double deltaR = (newREncoder - oldREndcoder) * encoderToInches;
//        double deltaS = (newSEncoder - oldSEncoder) * encoderToInches;
//
//        double totalL = newLEncoder * encoderToInches;
//        double totalR = newREncoder * encoderToInches;
//        double totalS = newLEncoder * encoderToInches;
//
//        oldLEncoder = newLEncoder;
//        oldREndcoder = newREncoder;
//        oldSEncoder = newSEncoder;
//
//        worldAngle = (totalL - totalR)/13.5;
//        double deltaAngle = (deltaL - deltaR)/13.5;
//
//        double localX;
//        double localY;
//
//        if (deltaAngle == 0){
//            localY = deltaS;
//            localX = deltaR;
//        } else {
//            localY = 2 * Math.sin()
//        }
//
//    }

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

        circularOdometry(dLeftPod, dRightPod, dMecanumPod, leftPodNew, rightPodNew);
    }

    double podWidth = 13.5; // Distance between x odometry pods in inches
    /* Circular Odometry assumes that the movement that occurred between each update was in the form
       of an arc (except when the robot moves directly forward). Each time the math is run, the
       algorithm uses coordinate system (x',y') where x' is the direct forward direction between the
       last point and the new point. */
    public void circularOdometry (double dLeftPod, double dRightPod, double dMecanumPod, double leftPod, double rightPod) {
        // See documentation in engineering notebook for details regarding math.

        double dMecanumInches = dMecanumPod * moveScaleFactor;
        double dLeftInches = dLeftPod * moveScaleFactor;
        double dRightInches = dRightPod * moveScaleFactor;

        // Change in angle of robot
        double dTheta = (dLeftInches - dRightInches) * turnScaleFactor;

        double leftInches = leftPod * moveScaleFactor;
        double rightInches = rightPod * moveScaleFactor;
        // New angle of robot
        double newWorldAngle = (leftInches - rightInches) * turnScaleFactor;

        // Angle to convert coordinate systems by (from robot system to global)
        double conversionAngle = (newWorldAngle + worldAngle) / 2.0;

        // New angle of robot
        worldAngle = newWorldAngle;

        double dXPrime = dLeftInches;
        double dYPrime = dMecanumInches;

        if (dTheta != 0) {
            double centerArcLength = (leftInches + rightInches) / 2.0;

            dXPrime = 2 * (Math.sin(dTheta / 2.0) * (dMecanumInches / dTheta));
            dYPrime = 2 * (Math.sin(dTheta / 2.0) * (centerArcLength / dTheta));
        }

        worldX += (dXPrime * Math.cos(conversionAngle)) - (dYPrime * Math.sin(conversionAngle));
        worldY += (dXPrime * Math.sin(conversionAngle)) + (dYPrime * Math.cos(conversionAngle));
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

//    public void constantVelocityOdometry() {
//
//        double fLeftNEW = robot.getfLeft().getCurrentPosition();
//        double fRightNEW = robot.getfRight().getCurrentPosition();
//        double bLeftNEW = robot.getbLeft().getCurrentPosition();
//        double bRightNEW = robot.getbRight().getCurrentPosition();
//
//        // find robot position
//        double fl = 2 * Math.PI * (fLeftNEW - fLeftOLD) / robot.getEncoderPerRevolution();
//        double fr = 2 * Math.PI * (fRightNEW - fRightOLD) / robot.getEncoderPerRevolution();
//        double bl = 2 * Math.PI * (bLeftNEW - bLeftOLD) / robot.getEncoderPerRevolution();
//        double br = 2 * Math.PI * (bRightNEW - bRightOLD) / robot.getEncoderPerRevolution();
//
//        double xDeltaRobot = robot.getWheelRadius()/4 * (fl + bl + br + fr);
//        double yDeltaRobot = robot.getWheelRadius()/4 * (-fl + bl - br + fr);
//        double angleDeltaRobot = robot.getWheelRadius()/4 *(-fl/(robot.getL()+robot.getW()) - bl/(robot.getL()+robot.getW()) + br/(robot.getL()+robot.getW()) + fr/(robot.getL()+robot.getW()));
//
//        //converting to global frame
//        if (angleDeltaRobot == 0){
//            xPosGlobal += xDeltaRobot * Math.cos(angleGlobal) - yDeltaRobot * Math.sin(angleGlobal);
//            yPosGlobal += xDeltaRobot * Math.sin(angleGlobal) + yDeltaRobot * Math.cos(angleGlobal);
//        } else {
//            xPosGlobal += (Math.cos(angleGlobal) * Math.sin(angleDeltaRobot) - (Math.cos(angleDeltaRobot) - 1) * Math.sin(angleGlobal)) * xDeltaRobot / angleDeltaRobot + (Math.cos(angleGlobal) * (Math.cos(angleDeltaRobot) - 1) - Math.sin(angleGlobal) * Math.sin(angleDeltaRobot)) * yDeltaRobot / angleDeltaRobot;
//            yPosGlobal += ((Math.cos(angleDeltaRobot) - 1) * Math.sin(angleGlobal) + (Math.cos(angleGlobal)) * Math.sin(angleDeltaRobot)) * yDeltaRobot / angleDeltaRobot + (Math.cos(angleGlobal) * (Math.cos(angleDeltaRobot) - 1) + Math.sin(angleGlobal) * Math.sin(angleDeltaRobot)) * xDeltaRobot / angleDeltaRobot;
//        }
//
//        angleGlobal = angleWrap((robot.getWheelCircumference() * (fLeftNEW)/robot.getEncoderPerRevolution() - robot.getWheelCircumference() * (fRightNEW)/robot.getEncoderPerRevolution()) / 14 * 0.51428571428);
//
//        fLeftOLD = fLeftNEW;
//        fRightOLD = fRightNEW;
//        bLeftOLD = bLeftNEW;
//        bRightOLD = bRightNEW;
//
//        robot.getTelemetry().addLine("XPOS: " + xPosGlobal);
//        robot.getTelemetry().addLine("YPOS: " + yPosGlobal);
//        robot.getTelemetry().addLine("ANGPOS: " + Math.toDegrees(Math.toDegrees(angleGlobal)));
//        robot.getTelemetry().update();
//    }

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