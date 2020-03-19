package org.firstinspires.ftc.teamcode.Skystone.Modules;

import android.os.SystemClock;

import org.firstinspires.ftc.teamcode.Skystone.HardwareCollection;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class OdometryModule {

    private long startTime;
    private long lastStartTime;

    private double oldLeftPod;
    private double oldRightPod;
    private double oldMecanumPod;

    public double worldX;
    public double worldY;
    public double worldAngle;

    private final double MECANUM_ENCODER_DISTANCE_FROM_CENTER = -0.625;
    private final double LEFT_RIGHT_ENCODER_DISTANCE_FROM_CENTER = 6.834660152;
    private final double MOVE_SCALE_FACTOR = 0.004110264;
    private final double TURN_SCALE_FACTOR = 0.5/LEFT_RIGHT_ENCODER_DISTANCE_FROM_CENTER;

    public StringBuilder odometryData;

    public OdometryModule(){
        oldLeftPod = 0.0;
        oldRightPod = 0.0;
        oldMecanumPod = 0.0;

        worldX = 0.0;
        worldY = 0.0;
        worldAngle = 0.0;

        odometryData = new StringBuilder();
        odometryData.append("worldX worldY worldAngle oldLeftPod oldRightPod");
        odometryData.append("\n");
    }

    public synchronized void update(Robot robot, HardwareCollection hardwareCollection) {

        if (robot.isDebug){
            odometryData.append(worldX);
            odometryData.append(" ");
            odometryData.append(worldY);
            odometryData.append(" ");
            odometryData.append(worldAngle);
            odometryData.append(" ");
            odometryData.append(oldLeftPod);
            odometryData.append(" ");
            odometryData.append(oldRightPod);
            odometryData.append("\n");
        }

        lastStartTime = startTime;
        startTime = SystemClock.elapsedRealtime();
        robot.telemetry.addLine("update speed: " + (startTime - lastStartTime));

        double leftPodNew = -hardwareCollection.fLeft.getCurrentPosition();
        double rightPodNew = -hardwareCollection.fRight.getCurrentPosition();
        double mecanumPodNew = hardwareCollection.bLeft.getCurrentPosition();

        double dLeftPod = leftPodNew - oldLeftPod;
        double dRightPod = rightPodNew - oldRightPod;
        double dMecanumPod = mecanumPodNew - oldMecanumPod;

        circularOdometry(dLeftPod, dRightPod, dMecanumPod);

        oldLeftPod = leftPodNew;
        oldRightPod = rightPodNew;
        oldMecanumPod = mecanumPodNew;
    }

    /* Circular OdometryModule assumes that the movement that occurred between each update was in the form
       of an arc

       By assuming arcs, we are assuming that we are constantly changing heading and position between each iteration.
       We are assuming constant velocity and constant rate of turn.
    */
    public void circularOdometry(double dLeftPod, double dRightPod, double dMecanumPod) {
        double dLeftInches = dLeftPod * MOVE_SCALE_FACTOR;
        double dRightInches = dRightPod * MOVE_SCALE_FACTOR;
        double dMecanumInches = dMecanumPod * MOVE_SCALE_FACTOR;
        double dAngle = (dLeftInches - dRightInches) * TURN_SCALE_FACTOR;
        double currAngle = worldAngle;

        double dRobotX = (dRightInches + dAngle * LEFT_RIGHT_ENCODER_DISTANCE_FROM_CENTER) * sinXOverX(dAngle) + (dMecanumInches + dAngle * MECANUM_ENCODER_DISTANCE_FROM_CENTER) * oneMinusCosXOverX(dAngle);
        double dRobotY = (dMecanumInches + dAngle * MECANUM_ENCODER_DISTANCE_FROM_CENTER) * sinXOverX(dAngle) + (dRightInches + dAngle * LEFT_RIGHT_ENCODER_DISTANCE_FROM_CENTER) * oneMinusCosXOverX(dAngle);

        worldAngle = currAngle + dAngle;
        worldX += dRobotX * Math.cos(currAngle) - dRobotY * Math.sin(currAngle);
        worldY += dRobotX * Math.sin(currAngle) + dRobotY * Math.cos(currAngle);
    }

    public double sinXOverX(double x){
        // if it is within certain threshold, maclaurin/taylor series expansion
        double threshold = 1.0;
        if (Math.abs(x) < threshold){
            int cycles = 9;
            double result = 0;
            double numer = 1;
            double denom = 1;
            for (int i = 0; i < cycles; i++) {
                // see maclaurin/taylor series expansion for sin x, then divide by x
                result += numer / denom;
                numer *= -x * x;
                denom *= (2 * i + 2) * (2 * i + 3);
            }
            return result;
        } else {
            return Math.sin(x)/x;
        }
    }

    public double oneMinusCosXOverX(double x){
        // if it is within certain threshold, maclaurin/taylor series expansion
        // error approximation: 0.707 * 2^10 / 10! = 0.002, meaning worst case accuracy to 3 decimal places within threshold
        double threshold = 1.0;
        if (Math.abs(x) < threshold){
            int cycles = 9;
            double result = 0;
            double numer = x;
            double denom = 2;
            for (int i = 0; i < cycles; i++) {
                // see maclaurin/taylor series expansion for cos x, then manipulate
                result += numer / denom;
                numer *= -x * x;
                denom *= (2 * i + 3) * (2 * i + 4);
            }
            return result;
        } else {
            return (1-Math.cos(x))/x;
        }
    }

    public void resetOdometry() {
        oldLeftPod = 0;
        oldRightPod = 0;
        oldMecanumPod = 0;

        worldX = 0;
        worldY = 0;
        worldAngle = 0;
    }
}