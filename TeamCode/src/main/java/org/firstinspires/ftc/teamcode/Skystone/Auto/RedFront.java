package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Robot;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

import java.util.HashMap;

@Autonomous(name = "RedFront", group = "LinearOpmode")
public class RedFront extends AutoBase {
    @Override
    public void runOpMode() {
        long startTime;
        initLogic();

        waitForStart();

        startTime = SystemClock.elapsedRealtime();

        // Positions assuming center Skystone
        double firstSkystoneY = -2;
        double secondSkyStoneY = -19;
        double secondSkyStoneX = 45;
        double thirdStoneY = -24.5;
        double thirdStoneX = 33;
        double anglelock = 20;
        Vision.Location skystoneLocation = Vision.Location.UNKNOWN;

        try {
            skystoneLocation = vision.runDetection(true, true);
        } catch (InterruptedException e) {
            // If detection is interrupted, program will have been exited.
            Log.d("vision.runDetection","interrupted");
        }

        telemetry.addLine("Detection Result: " + skystoneLocation.toString());
        telemetry.update();

        // why?
        sleep(250);

        position2D.startOdometry();

        // Change Skystone positions if detected left or right
        if (skystoneLocation == Vision.Location.LEFT) {
            firstSkystoneY = -7;
            secondSkyStoneY = -32;
            secondSkyStoneX = 52;
            anglelock = 20;
            thirdStoneX = 45;
            thirdStoneY = 2;
        } else if (skystoneLocation == Vision.Location.RIGHT) {
            firstSkystoneY = 4.5;
            secondSkyStoneY = -10;
            secondSkyStoneX = 45;
            thirdStoneY = -22;
        }

        double[][] toFirstStone = {
                {0, 0, 10, 0},
                {10, firstSkystoneY, 10, 0},
                {45, firstSkystoneY, 10, 0}};
        HashMap<Point, Robot.Actions> toFirstStoneActions = new HashMap<Point, Robot.Actions>() {{
            put(new Point(0, 0), Robot.Actions.START_INTAKE);
        }};

        double[][] toFoundation = {
                toFirstStone[toFirstStone.length - 1],
                {31, firstSkystoneY, 0, 10},
                {29, 17, -10, 20},
                {26, 20, -10, 20},
                {24, 30, -10, 20},
                {22, 43, -10, 20},
                {22, 55, 0, 20},
                {22, 67, 0, 20},
                {22, 68, 0, 20},
                {21, 74, 0, 20},
                {31, 83, 0, 10}};
        HashMap<Point, Robot.Actions> toFoundationActions = new HashMap<Point, Robot.Actions>() {{
            put(new Point(24, 30), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(24, 30), Robot.Actions.STOP_INTAKE);
        }};

        double[][] toSecondStone = {
                {toFoundation[toFoundation.length - 1][0], toFoundation[toFoundation.length - 1][1], -10, 0},
                {9, 63, 10, 0},
                {5, 60, 10, 0},
                {22, 61, -10, 0},
                {20, 29, 0, -10},
                {20, secondSkyStoneY + 5, 0, 10},
                {secondSkyStoneX, secondSkyStoneY, 30, 0}};
        HashMap<Point, Robot.Actions> toSecondStoneActions = new HashMap<Point, Robot.Actions>() {{
            put(new Point(22, 73), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(10, 70), Robot.Actions.RELEASE_FOUNDATION);
            put(new Point(28, -10), Robot.Actions.START_INTAKE);
        }};

        double[][] toDepositSecondStone = {
                {toSecondStone[toSecondStone.length - 1][0], toSecondStone[toSecondStone.length - 1][1], -10, 0},
                {secondSkyStoneX - 10, secondSkyStoneY + 10, -10, 0},
                {secondSkyStoneX - 5, secondSkyStoneY + 8, -10, 0},
                {25, 29, 0, 20},
                {23, 60, 0, 10},
                {23, 62, 0, 10},
                {14, 66, 0, 10}};
        HashMap<Point, Robot.Actions> toDepositSecondStoneActions = new HashMap<Point, Robot.Actions>() {{
            put(new Point(28, 20), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(35, -3), Robot.Actions.STOP_INTAKE);
            put(new Point(35, 15), Robot.Actions.START_INTAKE);
        }};

        double[][] toThirdStone = {
                toDepositSecondStone[toDepositSecondStone.length - 1],
                {21, 63, 5, 10},
                {21, 60, 0, -10},
                {21, 30, 0, -10},
                {21, 10, 0, -10},
                {32, 6, 0, -10},
                {thirdStoneX, thirdStoneY, 10, 0}};
        HashMap<Point, Robot.Actions> toThirdStoneActions = new HashMap<Point, Robot.Actions>() {{
            put(new Point(22, 58), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(28, 30), Robot.Actions.START_INTAKE);
        }};

        double[][] toDepositThirdStone = {
                toThirdStone[toThirdStone.length - 1],
                {27, -15, 0, -10},
                {25, 29, 0, 20},
                {25, 61, 0, 10},
                {18, 66, 0, 10}};
        HashMap<Point, Robot.Actions> toParkAfterThirdStoneActions = new HashMap<Point, Robot.Actions>() {{
            put(new Point(23, 15), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(25, 0), Robot.Actions.STOP_INTAKE);
            put(new Point(25, 15), Robot.Actions.START_INTAKE);
        }};


        double[][] toPark = {
                {toDepositThirdStone[toDepositThirdStone.length - 1][0], toDepositThirdStone[toDepositThirdStone.length - 1][1], 0, -10},
                {22, 29, 0, -10}};
        HashMap<Point, Robot.Actions> toParkActions = new HashMap<Point, Robot.Actions>() {{
            put(new Point(25, 65), Robot.Actions.RETRACT_OUTTAKE);
        }};

        intake(true);
        robot.splineMove(toFirstStone, 0.5, 1, 0.4, 3, 0, 0, 20,
                toFirstStoneActions, true, 2000);

        robot.splineMove(toFoundation, 1, 1, 0.5, 14, Math.toRadians(180), Math.toRadians(180), 25,
                toFoundationActions, true, 4250);

        // get ready to pull foundation
//        telemetry.addLine("X: " + robot.getRobotPos().x);
//        telemetry.addLine("X: " + robot.getRobotPos().y);
//        telemetry.addLine("Angle: " + robot.getAnglePos());
//        telemetry.update();

        robot.foundationMovers(true);
        sleep(350);

        robot.splineMove(toSecondStone, 1, 1, 1, 20, 0, Math.toRadians(300), anglelock,
                toSecondStoneActions, true, 8000);

        robot.splineMove(toDepositSecondStone, 1, 1, 0.5, 14, Math.toRadians(180), Math.toRadians(270), 18,
                toDepositSecondStoneActions, true, 4000);

        robot.foundationMovers(false);
        robot.getClamp().setPosition(robot.CLAMP_SERVO_RELEASED);
        robot.brakeRobot();

        if (SystemClock.elapsedRealtime() - startTime < 25000) {
            robot.splineMove(toThirdStone, 0.7, 1, 0.65, 20, 0, Math.toRadians(270), 20,
                    toThirdStoneActions, true, 6000);
            robot.splineMove(toDepositThirdStone, 1, 1, 0.3, 10, Math.toRadians(180), Math.toRadians(270), 20, toParkAfterThirdStoneActions, true, 4000);
            robot.foundationMovers(false);
            robot.splineMove(toPark, 0.6, 1, 0.4, 5, 0, Math.toRadians(270), 5, toParkActions);
        } else {
            robot.splineMove(toPark, 0.6, 1, 0.4, 5, 0, Math.toRadians(270), 5, toParkActions);
        }
    }
}