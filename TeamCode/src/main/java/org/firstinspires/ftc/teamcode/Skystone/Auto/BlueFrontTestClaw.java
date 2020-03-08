package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Action;
import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Enums.ActionType;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

import java.util.ArrayList;

@Autonomous(name = "BlueFrontTestClaw", group = "LinearOpmode")
public class BlueFrontTestClaw extends AutoBase {
    @Override
    public void runOpMode() {
        long startTime;
        initLogic();

        waitForStart();
        startTime = SystemClock.elapsedRealtime();

        // Positions assuming center Skystone
        double firstSkystoneY = 3;
        double secondSkyStoneY = 16;
        double thirdStoneY = 25;
        double thirdStoneX = 57;
        double anglelock = 27;
        double angleLockThird = 55;
        double thirdStoneXPath = 50;
        double mecanumPoint = 5;
        int foundation  = 4500;

        Vision.Location skystoneLocation = Vision.Location.UNKNOWN;

        try {
            skystoneLocation = vision.runDetection(false, false);
        } catch (Exception e) {

        }

        telemetry.addLine("Detection Result: " + skystoneLocation.toString());
        telemetry.update();

        sleep(250);

        position2D.startOdometry();

        // Change Skystone positions if detected left or right
        if (skystoneLocation == Vision.Location.LEFT) {
            firstSkystoneY = -7;
            secondSkyStoneY = 3.5;
            anglelock = 30;
            thirdStoneX = 56;
            thirdStoneY = 16;
            angleLockThird = 70;
            thirdStoneXPath = 54;
            mecanumPoint = -2;
        } else if (skystoneLocation == Vision.Location.RIGHT) {
            firstSkystoneY = 15;
            secondSkyStoneY = 23.5;
            anglelock = 30;
            thirdStoneX = 73;
            thirdStoneY = 24;
            foundation = 4750;
            angleLockThird = 35;
        }

        double[][] toFirstStone = {
                {0, 0},
                {48, firstSkystoneY}};
        ArrayList<Action> toFirstStoneActions = new ArrayList<Action>();
        toFirstStoneActions.add(new Action(ActionType.START_INTAKE, new Point(0, 0), robot));

        double[][] toFoundation = {
                toFirstStone[toFirstStone.length - 1],
                {35, firstSkystoneY - 2},
                {32, -17},
                {31, -20},
                {31, -30},
                {29, -43},
                {27, -55},
                {27, -60},
                {27, -64},
                {27, -66},
                {29, -71},
                {41, -85}};
        ArrayList<Action> toFoundationActions = new ArrayList<Action>();
        toFoundationActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(30, -15), robot, 50));
        toFoundationActions.add(new Action(ActionType.EXTEND_FOUNDATION, robot, true));
        toFoundationActions.add(new Action(ActionType.STOP_INTAKE, new Point(28, -7), robot));

        double[][] toReleaseFoundation = {
                {toFoundation[toFoundation.length - 1][0], toFoundation[toFoundation.length - 1][1], -10, 0},
                {27, -70},
                {19, -66},
        };
        ArrayList<Action> toReleaseFoundationActions = new ArrayList<Action>();
        toReleaseFoundationActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE_FRONT_CLAW_REMAIN, new Point(33, -73), robot, 0));
        toReleaseFoundationActions.add(new Action(ActionType.RELEASE_FOUNDATION, robot, true));

        double[][] toSecondStone = {
                {toReleaseFoundation[toReleaseFoundation.length - 1][0], toReleaseFoundation[toReleaseFoundation.length - 1][1], -10, 0},
                {27, -60},
                {31, -29},
                {31, secondSkyStoneY-15},
                {53, secondSkyStoneY},
                {51, secondSkyStoneY + 2.5}};
        ArrayList<Action> toSecondStoneActions = new ArrayList<Action>();
        toSecondStoneActions.add(new Action(ActionType.START_INTAKE, new Point(35, 0), robot));

        double[][] toDepositSecondStone = {
                toSecondStone[toSecondStone.length - 1],
                {37, secondSkyStoneY - 10},
                {37, -29},
                {37, -47},
                {37, -50},
                {37, -78}};
        ArrayList<Action> toDepositSecondStoneActions = new ArrayList<Action>();
        toDepositSecondStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(35, -5), robot, 375));
        toDepositSecondStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(35, 0), robot));

        double[][] toThirdStone = {
                toDepositSecondStone[toDepositSecondStone.length - 1],
                {38, -45},
                {39, -40},
                {40, -30},
                {48, -15},
                {thirdStoneXPath, 5},
                {thirdStoneXPath + 9, 0},
                {thirdStoneX, thirdStoneY - 10},
                {thirdStoneX, thirdStoneY}};
        ArrayList<Action> toThirdStoneActions = new ArrayList<Action>();
        toThirdStoneActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE_FRONT_CLAW_REMAIN, new Point(38, -69), robot, 0));
        toThirdStoneActions.add(new Action(ActionType.START_INTAKE, new Point(43, 0), robot));

        double[][] toDepositThirdStone = {
                toThirdStone[toThirdStone.length - 1],
                {40, 5},
                {40, -5},
                {40, -15},
                {40, -20},
                {40, -29},
                {40, -35},
                {40, -45},
                {40, -55},
                {40, -65},
                {40, -80}};
        ArrayList<Action> toParkAfterThirdStoneActions = new ArrayList<Action>();
        toParkAfterThirdStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(40, -8), robot, 375));
        toParkAfterThirdStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(toThirdStone[toThirdStone.length - 1][0] - 15, toThirdStone[toThirdStone.length - 1][1] - 15), robot));

        double[][] toPark = {
                {toDepositThirdStone[toDepositThirdStone.length - 1][0], toDepositThirdStone[toDepositThirdStone.length - 1][1], 0, -10},
                {42, -55},
                {42, -35}};
        ArrayList<Action> toParkActions = new ArrayList<Action>();
        toParkActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE_FRONT_CLAW_REMAIN, new Point(35, -74), robot, 0));

        robot.splineMove(toFirstStone, 0.65, 0.5, .7, 35, 0, Math.toRadians(0), 0,
                toFirstStoneActions, true, 2250);

        robot.dumpPoints("" + startTime, "1");

        robot.splineMove(toFoundation, 1, 1, 0.3, 30, Math.toRadians(180), Math.toRadians(180), 25,
                toFoundationActions, true, foundation, true, new Point(20, 0));

        robot.foundationMovers(true);

        robot.getLinearOpMode().sleep(150); // Allow foundation movers to deploy

        robot.dumpPoints("" + startTime, "2");

        robot.splineMove(toReleaseFoundation, 1, 1, 1, 5, 0, Math.toRadians(90), 12,
                toReleaseFoundationActions, true, 2250);

        robot.foundationMovers(false);

        robot.getLinearOpMode().sleep(150); // Allow foundation movers to deploy

        robot.dumpPoints("" + startTime, "3");

        robot.splineMove(toSecondStone, 1, 1, 0.3, 1.5, 0, Math.toRadians(70), anglelock,
                toSecondStoneActions, true, 5500);

        robot.dumpPoints("" + startTime, "4");

        robot.splineMove(toDepositSecondStone, 1, 1, 0.4, 35, Math.toRadians(180), Math.toRadians(90), 95,
                toDepositSecondStoneActions, true, 4500, true, new Point(35, mecanumPoint));

        robot.getBackClamp().setPosition(robot.BACKCLAMP_RELEASED);

        robot.getLinearOpMode().sleep(250); // Wait to finish releasing foundation

        robot.dumpPoints("" + startTime, "5");

        robot.splineMove(toThirdStone, 1, 1, 0.5, 10, 0, Math.toRadians(90), angleLockThird,
                toThirdStoneActions, true, 4500);

        robot.dumpPoints("" + startTime, "6");

        robot.splineMove(toDepositThirdStone, 1, 1, .4, 35, Math.toRadians(180), Math.toRadians(90), 95, toParkAfterThirdStoneActions, true, 4500, true, new Point(41, mecanumPoint + 10));

        robot.getBackClamp().setPosition(robot.BACKCLAMP_RELEASED);

        robot.getLinearOpMode().sleep(250); // Wait to finish releasing foundation

        robot.dumpPoints("" + startTime, "7");

        robot.splineMove(toPark, .65, 1, 0.4, 10, 0, Math.toRadians(90), 35, toParkActions, true, 3000);

        robot.dumpPoints("" + startTime, "8");
        robot.brakeRobot();
    }
}