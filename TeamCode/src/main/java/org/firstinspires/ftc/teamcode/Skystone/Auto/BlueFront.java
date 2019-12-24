package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Robot;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

import java.util.HashMap;

@Autonomous(name = "BlueFront", group = "LinearOpmode")
public class BlueFront extends AutoBase {
    @Override
    public void runOpMode() {
        long startTime;
        initLogic();

        waitForStart();
        startTime = SystemClock.elapsedRealtime();

        // Positions assuming center Skystone
        double firstSkystoneY = 2;
        double secondSkyStoneY = 13;
        double secondSkyStoneX = 45;
        double thirdStoneY = 24;
        double thirdStoneX = 52;
        double anglelock = 30;
        double angleLockAngle = Math.toRadians(60);

        Vision.Location skystoneLocation = Vision.Location.UNKNOWN;
        try {
            skystoneLocation = vision.runDetection(true, false);
        } catch (Exception e) {

        }

        telemetry.addLine("Detection Result: " + skystoneLocation.toString());
        telemetry.update();

        sleep(250);

        position2D.startOdometry();

        // Change Skystone positions if detected left or right
        if (skystoneLocation == Vision.Location.LEFT) {
            firstSkystoneY = -4.5;
            secondSkyStoneY = 9.75;
            secondSkyStoneX = 45;
            anglelock = 45;
            thirdStoneX = 33;
            thirdStoneY = 21;
        } else if (skystoneLocation == Vision.Location.RIGHT) {
            firstSkystoneY = 4;
            secondSkyStoneY = 25;
            secondSkyStoneX = 45;
            anglelock = 33;
            thirdStoneX = 70;
            thirdStoneY = 31.5;
            angleLockAngle = Math.toRadians(50);
        }

        double[][] toFirstStone = {
                {0, 0, 10, 0},
                {10, firstSkystoneY, 10, 0},
                {48, firstSkystoneY, 10, 0}};
        HashMap<Point, Robot.Actions> toFirstStoneActions = new HashMap<Point, Robot.Actions>() {{
            put(new Point(0, 0), Robot.Actions.START_INTAKE);
        }};

        double[][] toFoundation = {
                toFirstStone[toFirstStone.length - 1],
                {31, firstSkystoneY, 0, -10},
                {29, -17, -10, -20},
                {25, -20, -10, -20},
                {25, -30, -10, -20},
                {25, -43, -10, -20},
                {24, -55, 0, -20},
                {25, -67, 0, -20},
                {26, -68, 0, -20},
                {26, -74, 0, -20},
                {34.25, -83, 0, -10}};
        HashMap<Point, Robot.Actions> toFoundationActions = new HashMap<Point, Robot.Actions>() {{
            put(new Point(24, -26), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(24, -45), Robot.Actions.STOP_INTAKE);
        }};

        double[][] toSecondStone = {
                {toFoundation[toFoundation.length - 1][0], toFoundation[toFoundation.length - 1][1], -10, 0},
                {12, -63, 10, 0},
                {10, -60, 10, 0},
                {26, -61, -10, 0},
                {26, -29, 0, 10},
                {25, secondSkyStoneY - 10, 0, -10},
                {secondSkyStoneX, secondSkyStoneY, 30, 0}};
        HashMap<Point, Robot.Actions> toSecondStoneActions = new HashMap<Point, Robot.Actions>() {{
            put(new Point(22, -73), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(10, -70), Robot.Actions.RELEASE_FOUNDATION);
            put(new Point(28, 10), Robot.Actions.START_INTAKE);
        }};

        double[][] toDepositSecondStone = {
                {toSecondStone[toSecondStone.length - 1][0], toSecondStone[toSecondStone.length - 1][1], -10, 0},
                {secondSkyStoneX - 2, secondSkyStoneY - 10, -10, 0},
                {secondSkyStoneX - 7, secondSkyStoneY - 8, -10, 0},
                {38, 0, 0, -20},
                {36, -29, 0, -20},
                {36, -64, 0, -10},
                {29, -65, 0, -10},
                {28, -68, 0, -10}};
        HashMap<Point, Robot.Actions> toDepositSecondStoneActions = new HashMap<Point, Robot.Actions>() {{
            put(new Point(28, -15), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(35, -15), Robot.Actions.STOP_INTAKE);
            put(new Point(35, -25), Robot.Actions.START_INTAKE);
        }};

        final double[][] toThirdStone = {
                toDepositSecondStone[toDepositSecondStone.length - 1],
                {37, -63, 5, -10},
                {34, -60, 0, 10},
                {34, -30, 0, 10},
                {34, -10, 0, -10},
                {43, -6, 0, 10},
                {thirdStoneX, thirdStoneY, 10, 0}};
        HashMap<Point, Robot.Actions> toThirdStoneActions = new HashMap<Point, Robot.Actions>() {{
            put(new Point(22, -58), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(28, -30), Robot.Actions.START_INTAKE);
        }};

        double[][] toDepositThirdStone = {
                toThirdStone[toThirdStone.length - 1],
                {46, 15, 0, 10},
                {45, -29, 0, -20},
                {45, -61, 0, -10},
                {45, -65, 0, -10}};
        HashMap<Point, Robot.Actions> toParkAfterThirdStoneActions = new HashMap<Point, Robot.Actions>() {{
            put(new Point(23, 0), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(toThirdStone[toThirdStone.length - 1][0] - 15, toThirdStone[toThirdStone.length - 1][1] - 25), Robot.Actions.STOP_INTAKE);
            put(new Point(21, -20), Robot.Actions.START_INTAKE);
        }};

        double[][] toDepositThirdStoneOtherwise = {
                toThirdStone[toThirdStone.length - 1],
                {44, 15, 0, 10},
                {42, -29, 0, -20},
                {42, -61, 0, -10},
                {42, -65, 0, -10}};
        HashMap<Point, Robot.Actions> toParkAfterThirdStoneActionsOtherwise = new HashMap<Point, Robot.Actions>() {{
            put(new Point(23, 0), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(toThirdStone[toThirdStone.length - 1][0] - 15, toThirdStone[toThirdStone.length - 1][1] - 25), Robot.Actions.STOP_INTAKE);
            put(new Point(21, -20), Robot.Actions.START_INTAKE);
        }};

        double[][] toPark = {
                {toDepositThirdStone[toDepositThirdStone.length - 1][0], toDepositThirdStone[toDepositThirdStone.length - 1][1], 0, 10},
                {37, -55, 0, 10},
                {37, -36, 0, 10}};
        HashMap<Point, Robot.Actions> toParkActions = new HashMap<Point, Robot.Actions>() {{
            put(new Point(25, -65), Robot.Actions.RETRACT_OUTTAKE);
        }};

        double[][] toParkDitch = {
                {toThirdStone[toThirdStone.length - 1][0], toThirdStone[toThirdStone.length - 1][1], -10, -10},
                {37, toThirdStone[toThirdStone.length - 1][1] - 20, -10, -10},
                {37, -36, -10, -10}};
        HashMap<Point, Robot.Actions> toParkDitchActions = new HashMap<Point, Robot.Actions>() {{
            put(new Point(25, -65), Robot.Actions.RETRACT_OUTTAKE);
        }};

        intake(true);
        robot.splineMove(toFirstStone, 0.6, 1, 0.55, 35, 0, 0, 20,
                toFirstStoneActions, true, 3000);
        //to first stone is 1
        robot.dumpPoints("" + startTime, "1");

        robot.splineMove(toFoundation, 1, 1, 0.4, 20, Math.toRadians(180), Math.toRadians(180), 25,
                toFoundationActions, true, 4750);
        // to foundation is 2
        robot.dumpPoints("" + startTime, "2");

        // get ready to pull foundation
        robot.foundationMovers(true);
        sleep(350);

        robot.splineMove(toSecondStone, 1, 1, 0.7, 20, 0, angleLockAngle, anglelock,
                toSecondStoneActions, true, 6500);
        //to second stone is 3
        robot.dumpPoints("" + startTime, "3");

        robot.splineMove(toDepositSecondStone, 1, 1, 0.5, 35, Math.toRadians(180), Math.toRadians(90), 18,
                toDepositSecondStoneActions, true, 4500);
        //to deposit second stone is 4
        robot.dumpPoints("" + startTime, "4");

        robot.foundationMovers(false);
        robot.getClamp().setPosition(robot.CLAMP_SERVO_RELEASED);
        robot.brakeRobot();

        robot.splineMove(toThirdStone, 0.5, 1, 0.8, 70, 0, Math.toRadians(90), 20,
                toThirdStoneActions, true, 4500);
        //to thrid stone is 5
        robot.dumpPoints("" + startTime, "5");
        if (true || SystemClock.elapsedRealtime() - startTime < 26000) {
            if(skystoneLocation == Vision.Location.RIGHT) {
                robot.splineMove(toDepositThirdStone, 1, 1, 0.3, 30, Math.toRadians(180), Math.toRadians(90), 20, toParkAfterThirdStoneActions, true, 4250);
            }else{
                robot.splineMove(toDepositThirdStoneOtherwise, 1, 1, 0.3, 30, Math.toRadians(180), Math.toRadians(90), 20, toParkAfterThirdStoneActionsOtherwise, true, 4250);
            }
            //to deposit third stone is 6
            robot.dumpPoints("" + startTime, "6");

            robot.foundationMovers(false);
            robot.splineMove(toPark, 0.5, 1, 0.3, 10, 0, Math.toRadians(90), 5, toParkActions);

            //to park is 7
            robot.dumpPoints("" + startTime, "7");
        } else {
            robot.splineMove(toParkDitch, 0.6, 1, 0.55, 17, Math.toRadians(180), Math.toRadians(90), 5, toParkDitchActions);

            //to park is 7
            robot.dumpPoints("" + startTime, "7");
        }
    }
}