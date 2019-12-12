package org.firstinspires.ftc.teamcode.Skystone.Odometry;

import android.os.AsyncTask;

import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class Position2D{
    Robot robot;
    NewThread newThread;
    public Position2D(Robot robot) {
        this.robot = robot;
        Odometry o = new Odometry(robot);
        newThread = new NewThread(robot,o);
    }

    public void startOdometry(){
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
        while(robot.getLinearOpMode().opModeIsActive()) {
            o.circularOdometry();
            robot.setRobotPos(new Point(o.worldX, o.worldY));
            robot.setAnglePos(o.worldAngle);
        }
        return true;
    }

    protected void onPostExecute(Boolean result) {
        if(result) {
            robot.getTelemetry().addLine("DONE");
            robot.getTelemetry().update();
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

    double moveScaleFactor = 0.004177098;
    double turnScaleFactor = 0.000301686;
    double strafeScaleFactor = 0.004135465;
    double strafePredictionScalingFactor = 0.092;

//    double encoderToInches = ;

    public Odometry(Robot robot){
        this.robot = robot;
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

    public void circularOdometry () {
        double leftPodNew = -1 * robot.getfLeft().getCurrentPosition(); // fix this for new odo config
        double rightPodNew = -1 * robot.getfRight().getCurrentPosition(); //fix this for new odo config
        double mecanumPodNew = -1 * robot.getbLeft().getCurrentPosition(); // fix this for new odo config

        double leftIncrement = (leftPodNew-leftPodOld) * moveScaleFactor;
        double rightIncrement = (rightPodNew - rightPodOld) * moveScaleFactor;
        double mecanumIncrement = (mecanumPodNew - mecanumPodOld) * strafeScaleFactor;
        double angleIncrement = (leftIncrement - rightIncrement) * turnScaleFactor;
        double mecanumIncrementPrediction = Math.toDegrees(angleIncrement)*(strafePredictionScalingFactor);

        double worldAngleOld = worldAngle;
        worldAngle = (leftPodNew - rightPodNew) * turnScaleFactor;

        double updatedMecanumDistance = mecanumIncrement -  mecanumIncrementPrediction;

        double relativeY = updatedMecanumDistance;
        double relativeX = (leftIncrement + rightIncrement)/2;

        if (angleIncrement != 0.0){
            double radiusOfMovement = (rightIncrement + leftIncrement)/(2 * angleIncrement);
            double radiusOfStrafe = updatedMecanumDistance/angleIncrement;

            relativeY = radiusOfMovement * (1 - Math.cos(angleIncrement)) + radiusOfStrafe * Math.sin(angleIncrement);
            relativeX = radiusOfMovement * Math.sin(angleIncrement) + radiusOfStrafe * (1-Math.cos(angleIncrement));
        }

        worldY += (relativeY * Math.cos(worldAngleOld) + relativeX * Math.sin(worldAngleOld));
        worldX += relativeX * Math.cos(worldAngleOld) - relativeY * Math.sin(worldAngleOld);

        leftPodOld = leftPodNew;
        rightPodOld = rightPodNew;
        mecanumPodOld = mecanumPodNew;

//        robot.getTelemetry().addLine("(" + robot.getRobotPos().x + ", " + robot.getRobotPos().y + ")");
//        robot.getTelemetry().addLine("angle: " + Math.toDegrees(robot.getAnglePos()));
//        robot.getTelemetry().update();
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
}