package org.firstinspires.ftc.teamcode.Skystone.Odometry;

import android.os.AsyncTask;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.Skystone.MathFunctions;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

import static org.firstinspires.ftc.teamcode.Skystone.MathFunctions.angleWrap;

public class Position2D{
    FtcRobotControllerActivity activity;
    Robot robot;
    public Position2D(Robot robot) {
        this.robot = robot;
    }

    public void startOdometry(){
        Odometry o = new Odometry(robot);
        NewThread newThread = new NewThread(robot,o);

        newThread.execute();
    }
}
class NewThread extends AsyncTask<Void, Boolean, Boolean> {
    Robot robot;
    Odometry o;
    public NewThread(Robot robot, Odometry o){
        this.robot = robot;
        this.o = o;
    }

    @Override
    protected Boolean doInBackground(Void... params) {
        while(robot.linearOpMode.opModeIsActive()) {
            //o.constantVelocityOdometry();
            o.circularOdometry();
//            robot.robotPos.x = o.xPosGlobal;
//            robot.robotPos.y = o.yPosGlobal;
//            robot.anglePos = o.angleGlobal;
            robot.robotPos.x = o.worldX;
            robot.robotPos.y = o.worldY;
            robot.anglePos = o.worldAngle;
        }
        return true;
    }

    protected void onPostExecute(Boolean result) {
        if(result) {
            robot.telemetry.addLine("DONE");
            robot.telemetry.update();
        }
    }

}

class Odometry{

    public Robot robot;

    //for constant velo

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

    double moveScaleFactor = 0.00437516512;
    double turnScaleFactor = 0.00030885662;
    double strafeScaleFactor = 0.00408585365;
    double strafePredictionScalingFactor = 0.93;

    public Odometry(Robot robot){
        this.robot = robot;
        robot.intializeIMU();
    }

    public void circularOdometry () {
        double leftPodNew = -1 * robot.fLeft.getCurrentPosition(); // fix this for new odo config
        double rightPodNew = -1 * robot.fRight.getCurrentPosition(); //fix this for new odo config
        double mecanumPodNew = robot.bLeft.getCurrentPosition(); // fix this for new odo config

        double leftIncrement = (leftPodNew-leftPodOld) * moveScaleFactor;
        double rightIncrement = (rightPodNew - rightPodOld) * moveScaleFactor;
        double mecanumIncrement = (mecanumPodNew - mecanumPodOld) * strafeScaleFactor;
        double angleIncrement = (leftIncrement - rightIncrement) * turnScaleFactor;
        double mecanumIncrementPrediction = Math.toDegrees(angleIncrement)*(strafePredictionScalingFactor/10.0);

        double actualRealKindaMecanumDistance = mecanumIncrement -  mecanumIncrementPrediction;

        double relativeX = actualRealKindaMecanumDistance;
        double relativeY = (leftIncrement - rightIncrement)/2;

        if (angleIncrement != 0.0){
            double radiusOfMovement = (rightIncrement + leftIncrement)/(2 * angleIncrement);
            double radiusOfStrafe = actualRealKindaMecanumDistance/angleIncrement;

            relativeX = radiusOfMovement * (1 - Math.cos(angleIncrement)) + radiusOfStrafe * Math.sin(angleIncrement);
            relativeY = radiusOfMovement * Math.sin(angleIncrement) - radiusOfStrafe * (1-Math.cos(angleIncrement)); // maybe try + if doesnt work
        }

        worldX += relativeY * Math.cos(worldAngle) + relativeX * Math.sin(worldAngle);
        worldY += relativeY * Math.sin(worldAngle) - relativeX * Math.cos(worldAngle);
        worldAngle  = (leftPodNew - rightPodNew)  * turnScaleFactor;

        leftPodOld =leftPodNew;
        rightPodOld = rightPodNew;
        mecanumPodOld = mecanumPodNew;

        robot.telemetry.addLine("left x odo " + leftPodNew);
        robot.telemetry.addLine("right x odo " + rightPodNew);
        robot.telemetry.addLine("mecanum odo " + mecanumPodNew);
        robot.telemetry.addLine("XPOS: " + robot.robotPos.x);
        robot.telemetry.addLine("YPOS: " + robot.robotPos.y);
        robot.telemetry.addLine("ANGPOS: " + Math.toDegrees(robot.anglePos));
        robot.telemetry.update();
    }

    public void constantVelocityOdometry() {

        double fLeftNEW = robot.fLeft.getCurrentPosition();
        double fRightNEW = robot.fRight.getCurrentPosition();
        double bLeftNEW = robot.bLeft.getCurrentPosition();
        double bRightNEW = robot.bRight.getCurrentPosition();

        // find robot position
        double fl = 2 * Math.PI * (fLeftNEW - fLeftOLD) / robot.encoderPerRevolution;
        double fr = 2 * Math.PI * (fRightNEW - fRightOLD) / robot.encoderPerRevolution;
        double bl = 2 * Math.PI * (bLeftNEW - bLeftOLD) / robot.encoderPerRevolution;
        double br = 2 * Math.PI * (bRightNEW - bRightOLD) / robot.encoderPerRevolution;

        double xDeltaRobot = robot.wheelRadius/4 * (fl + bl + br + fr);
        double yDeltaRobot = robot.wheelRadius/4 * (-fl + bl - br + fr);
        double angleDeltaRobot = robot.wheelRadius/4 *(-fl/(robot.l+robot.w) - bl/(robot.l+robot.w) + br/(robot.l+robot.w) + fr/(robot.l+robot.w));

        //converting to global frame
        if (angleDeltaRobot == 0){
            xPosGlobal += xDeltaRobot * Math.cos(angleGlobal) - yDeltaRobot * Math.sin(angleGlobal);
            yPosGlobal += xDeltaRobot * Math.sin(angleGlobal) + yDeltaRobot * Math.cos(angleGlobal);
        } else {
            xPosGlobal += (Math.cos(angleGlobal) * Math.sin(angleDeltaRobot) - (Math.cos(angleDeltaRobot) - 1) * Math.sin(angleGlobal)) * xDeltaRobot / angleDeltaRobot + (Math.cos(angleGlobal) * (Math.cos(angleDeltaRobot) - 1) - Math.sin(angleGlobal) * Math.sin(angleDeltaRobot)) * yDeltaRobot / angleDeltaRobot;
            yPosGlobal += ((Math.cos(angleDeltaRobot) - 1) * Math.sin(angleGlobal) + (Math.cos(angleGlobal)) * Math.sin(angleDeltaRobot)) * yDeltaRobot / angleDeltaRobot + (Math.cos(angleGlobal) * (Math.cos(angleDeltaRobot) - 1) + Math.sin(angleGlobal) * Math.sin(angleDeltaRobot)) * xDeltaRobot / angleDeltaRobot;
        }

        angleGlobal = angleWrap((robot.wheelCircumference * (fLeftNEW)/robot.encoderPerRevolution - robot.wheelCircumference * (fRightNEW)/robot.encoderPerRevolution) / 14 * 0.51428571428);

        fLeftOLD = fLeftNEW;
        fRightOLD = fRightNEW;
        bLeftOLD = bLeftNEW;
        bRightOLD = bRightNEW;

        robot.telemetry.addLine("XPOS: " + xPosGlobal);
        robot.telemetry.addLine("YPOS: " + yPosGlobal);
        robot.telemetry.addLine("ANGPOS: " + Math.toDegrees(Math.toDegrees(angleGlobal)));
        robot.telemetry.update();
    }
}