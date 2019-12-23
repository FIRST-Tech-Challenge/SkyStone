package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Robot;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

import java.util.HashMap;

@Autonomous(name="RedFront2", group ="LinearOpmode")
public class RedFront extends AutoBase{
    @Override
    public void runOpMode() {
        long startTime;
        initLogic();

        waitForStart();
        startTime = SystemClock.elapsedRealtime();

        // Positions assuming center Skystone
        double firstSkystoneY = -2;
        double secondSkyStoneY = -25;
        double secondSkyStoneX = 55;
        double thirdStoneY = -20;
        double thirdStoneX = 32;
        double anglelock = 36;
        Vision.Location skystoneLocation = Vision.Location.UNKNOWN;
        try {
            skystoneLocation = vision.runDetection(true, true);
        }catch (Exception e){

        }

        telemetry.addLine("Detection Result: " + skystoneLocation.toString());
        telemetry.update();

        sleep(250);

        position2D.startOdometry();

        // Change Skystone positions if detected left or right
        if (skystoneLocation == Vision.Location.LEFT){
            firstSkystoneY = -7;
            secondSkyStoneY = -32;
            secondSkyStoneX = 52;
            anglelock = 36;
            thirdStoneX = 45;
            thirdStoneY = 2;
        } else if (skystoneLocation == Vision.Location.RIGHT){
            firstSkystoneY = 4.5;
            secondSkyStoneY = -13;
            secondSkyStoneX = 56;
        }

        double[][] toFirstStone = {
                {0,0,10,0},
                {10,firstSkystoneY,10,0},
                {45,firstSkystoneY,10,0}};
        HashMap<Point,Robot.Actions> toFirstStoneActions = new HashMap<Point,Robot.Actions>(){{
            put(new Point(0,0), Robot.Actions.START_INTAKE);
        }};

        double[][] toFoundation = {
                {31,firstSkystoneY,-10,0},
                {26,17,0,10},
                {26,20,0,10},
                {26,30,0,10},
                {25,67,0,10},
                {21,72,0,10},
                {21,80,0,10},
                {30,82,0,10}};
        HashMap<Point,Robot.Actions> toFoundationActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(24,30), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(24,30), Robot.Actions.STOP_INTAKE);
        }};

        double[][] toSecondStone = {
                {30,83,-10,0},
                {9, 63,10,0},
                {5,60,10,0},
                {20,60,0,-10},
                {21,29,0,-10},
                {23,secondSkyStoneY + 5,0,10},
                {secondSkyStoneX,secondSkyStoneY,30,0}};
        HashMap<Point,Robot.Actions> toSecondStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(22,73), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(10,70), Robot.Actions.RELEASE_FOUNDATION);
            put(new Point(28,-10), Robot.Actions.START_INTAKE);
        }};

        double[][] toDepositSecondStone = {
                {secondSkyStoneX,secondSkyStoneY,10,0},
                {secondSkyStoneX-18,secondSkyStoneY+10,10,0},
                {secondSkyStoneX-10,secondSkyStoneY+8,10,0},
                {27,29,0,20},
                {28,60,0,10},
                {19,62,0,10},
                {12,66,0,10}};
        HashMap<Point,Robot.Actions> toDepositSecondStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(28,26), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(35,-10), Robot.Actions.STOP_INTAKE);
            put(new Point(35,10), Robot.Actions.START_INTAKE);
        }};

        double[][] toPark = {
                {23,65,0,-10},
                {19,29,0,-10}};
        HashMap<Point,Robot.Actions> toParkActions = new HashMap<Point,Robot.Actions>(){{
            put(new Point(25,65), Robot.Actions.RETRACT_OUTTAKE);
        }};

        double[][] toThirdStone = {
                {22,63,5,10},
                {23,60,0,-10},
                {23, 30, 0,-10},
                {23,10,0,-10},
                {33,6,0,-10},
                {thirdStoneX, thirdStoneY, 10,0}};
        HashMap<Point,Robot.Actions> toThirdStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(22,58), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(28,30), Robot.Actions.START_INTAKE);
        }};

        double[][] toDepositThirdStone = {
                {thirdStoneX, thirdStoneY, 10,0},
                {24,-10,0,-10},
                {24,29,0,20},
                {20,61,0,10},
                {15,66,0,10}};
        HashMap<Point,Robot.Actions> toParkAfterThirdStoneActions = new HashMap<Point,Robot.Actions>(){{
            put(new Point(23,5), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(42,8), Robot.Actions.STOP_INTAKE);
        }};

        intake(true);
        robot.splineMove(toFirstStone,0.5,1, 0.65,3,0,0,20,
                toFirstStoneActions, true, 5000);

        robot.splineMove(toFoundation,1,1, 0.4, 14, Math.toRadians(180),Math.toRadians(180),25,
                toFoundationActions, true, 4500);

        // get ready to pull foundation
        telemetry.addLine("X: " + robot.getRobotPos().x);
        telemetry.addLine("X: " + robot.getRobotPos().y);
        telemetry.addLine("Angle: " + robot.getAnglePos());
        telemetry.update();

        robot.foundationMovers(true);
        sleep(350);

        robot.splineMove(toSecondStone,1,1, 1, 20,0,Math.toRadians(300),anglelock,
                toSecondStoneActions, true, 8000);

        robot.splineMove(toDepositSecondStone,1,1, 0.5, 14, Math.toRadians(180),Math.toRadians(270),18,
                toDepositSecondStoneActions, true, 5000);

        robot.foundationMovers(false);
        robot.getClamp().setPosition(robot.CLAMP_SERVO_RELEASED);
        robot.brakeRobot();

        if (SystemClock.elapsedRealtime() - startTime < 25000){
            robot.splineMove(toThirdStone, 0.7,1, 0.65, 20,0,Math.toRadians(270),20,
                    toThirdStoneActions,true,6000);
            robot.splineMove(toDepositThirdStone, 1, 1, 0.3, 10, Math.toRadians(180), Math.toRadians(270), 20, toParkAfterThirdStoneActions);
            robot.foundationMovers(false);
            robot.splineMove(toPark, 0.6, 1, 0.4, 5, 0, Math.toRadians(270), 5, toParkActions);
        } else {
            robot.splineMove(toPark, 0.6, 1, 0.4, 5, 0, Math.toRadians(270), 5, toParkActions);
        }
    }
}