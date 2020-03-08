package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Action;
import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Enums.ActionType;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

import java.util.ArrayList;

@Autonomous(name = "RedFrontTestClaw", group = "LinearOpmode")
public class RedFrontTestClaw extends AutoBase {
    @Override
    public void runOpMode() {
        long startTime;
        initLogic();

        waitForStart();
        startTime = SystemClock.elapsedRealtime();

        // Positions assuming center Skystone
        double firstSkystoneY = -5;
        double secondSkyStoneY = -12;
        double thirdStoneY = -26.5;
        double thirdStoneX = 44;
        double anglelock = 30;
        double angleLockThird = 45;
        double thirdStoneXPath = 38;
        long foundationTimeKill = 4500;

        Vision.Location skystoneLocation = Vision.Location.UNKNOWN;

        try {
            skystoneLocation = vision.runDetection(false, true);
        } catch (Exception e) {

        }

        telemetry.addLine("Detection Result: " + skystoneLocation.toString());
        telemetry.update();

        sleep(250);

        position2D.startOdometry();

        // Change Skystone positions if detected left or right
        if (skystoneLocation == Vision.Location.LEFT) {
            firstSkystoneY = -15.5;
            secondSkyStoneY = -21;
            anglelock = 30;
            thirdStoneX = 58;
            thirdStoneY = -27;
            angleLockThird = 35;
            thirdStoneXPath = 24;
        } else if (skystoneLocation == Vision.Location.RIGHT) {
            firstSkystoneY = 10;
            secondSkyStoneY = -5.25;
            anglelock = 30;
            thirdStoneX = 44.5;
            thirdStoneY = -19;
            foundationTimeKill = 4250;
        }

        double[][] toFirstStone = {
                {0, 0, 10, 0},
                {48, firstSkystoneY, 10, 0}};
        ArrayList<Action> toFirstStoneActions = new ArrayList<Action>();
        toFirstStoneActions.add(new Action(ActionType.START_INTAKE, new Point(0, 0), robot));

        double[][] toFoundation = {
                toFirstStone[toFirstStone.length - 1],
                {32, firstSkystoneY + 2},
                {30, 17},
                {29, 20},
                {29, 30},
                {27, 43},
                {24, 55},
                {22, 60},
                {22, 64},
                {22, 66},
                {24, 71},
                {36, 81}};
        ArrayList<Action> toFoundationActions = new ArrayList<Action>();
        toFoundationActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(25, 15), robot, 50));
        toFoundationActions.add(new Action(ActionType.EXTEND_FOUNDATION, robot, true));
        toFoundationActions.add(new Action(ActionType.STOP_INTAKE, new Point(25, 0), robot));

        double[][] toReleaseFoundation = {
                {toFoundation[toFoundation.length - 1][0], toFoundation[toFoundation.length - 1][1], -10, 0},
                {20, 70},
                {12, 66},
        };
        ArrayList<Action> toReleaseFoundationActions = new ArrayList<Action>();
        toReleaseFoundationActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE_FRONT_CLAW_REMAIN, new Point(22, 73), robot, 0));
        toReleaseFoundationActions.add(new Action(ActionType.RELEASE_FOUNDATION, robot, true));

        double[][] toSecondStone = {
                {toReleaseFoundation[toReleaseFoundation.length - 1][0], toReleaseFoundation[toReleaseFoundation.length - 1][1], -10, 0},
                {22, 64},
                {25, 29},
                {29, secondSkyStoneY + 5},
                {47, secondSkyStoneY},
                {43, secondSkyStoneY - 6}};
        ArrayList<Action> toSecondStoneActions = new ArrayList<Action>();
        toSecondStoneActions.add(new Action(ActionType.START_INTAKE, new Point(20, -10), robot));

        double[][] toDepositSecondStone = {
                toSecondStone[toSecondStone.length - 1],
                {27, secondSkyStoneY + 14},
                {26, 29},
                {26, 47},
                {26, 50},
                {26, 76.5}};
        ArrayList<Action> toDepositSecondStoneActions = new ArrayList<Action>();
        toDepositSecondStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(28, 5), robot, 375));
        toDepositSecondStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(25, -5), robot));

        double[][] toThirdStone = {
                toDepositSecondStone[toDepositSecondStone.length - 1],
                {27, 45},
                {27, 40},
                {26, 30},
                {26, 15},
                {thirdStoneXPath, 10},
                {thirdStoneXPath + 4, 8},
                {thirdStoneX, thirdStoneY + 6},
                {thirdStoneX, thirdStoneY}};
        ArrayList<Action> toThirdStoneActions = new ArrayList<Action>();
        toThirdStoneActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE_FRONT_CLAW_REMAIN, new Point(25, 71), robot, 0));
        toThirdStoneActions.add(new Action(ActionType.START_INTAKE, new Point(25, 0), robot));

        double[][] toDepositThirdStone = {
                toThirdStone[toThirdStone.length - 1],
                {24, 0},
                {24, 5},
                {24, 15},
                {24, 20},
                {24, 29},
                {25, 35},
                {25, 45},
                {25, 55},
                {25, 65},
                {25, 76}};
        ArrayList<Action> toParkAfterThirdStoneActions = new ArrayList<Action>();
        toParkAfterThirdStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(25, 5), robot, 375));
        toParkAfterThirdStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(toThirdStone[toThirdStone.length - 1][0] - 15, toThirdStone[toThirdStone.length - 1][1] + 15), robot));

        double[][] toPark = {
                {toDepositThirdStone[toDepositThirdStone.length - 1][0], toDepositThirdStone[toDepositThirdStone.length - 1][1], 0, -10},
                {25, 55},
                {25, 36}};
        ArrayList<Action> toParkActions = new ArrayList<Action>();
        toParkActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE_FRONT_CLAW_REMAIN, new Point(25, 74), robot, 0));

        robot.splineMove(toFirstStone, 0.65, 0.5, 0.55, 35, 0, Math.toRadians(0), 0,
                toFirstStoneActions, true, 2500);

        robot.dumpPoints("" + startTime, "1");

        robot.splineMove(toFoundation, 1, 1, 0.55, 15, Math.toRadians(180), Math.toRadians(180), 30,
                toFoundationActions, true, foundationTimeKill, true, new Point(20, 5));

        robot.foundationMovers(true);

        robot.getLinearOpMode().sleep(150); // Allow foundation movers to deploy

        robot.dumpPoints("" + startTime, "2");

        robot.splineMove(toReleaseFoundation, 1, 1, 1, 5, 0, Math.toRadians(270), 12,
                toReleaseFoundationActions, true, 2250);
        robot.foundationMovers(false);

        robot.getLinearOpMode().sleep(150); // Allow foundation movers to deploy
        robot.dumpPoints("" + startTime, "3");

//        robot.getLinearOpMode().sleep(150); // Wait to finish releasing foundation

        robot.splineMove(toSecondStone, 1, 1, 1, 1, 0, Math.toRadians(290), anglelock,
                toSecondStoneActions, true, 5500);

        robot.dumpPoints("" + startTime, "4");

        robot.splineMove(toDepositSecondStone, 1, 1, 0.4, 35, Math.toRadians(180), Math.toRadians(270), 90,
                toDepositSecondStoneActions, true, 4500, true, new Point(20, 5));

        robot.getBackClamp().setPosition(robot.BACKCLAMP_RELEASED);

        robot.getLinearOpMode().sleep(250); // Wait to finish releasing foundation

        robot.dumpPoints("" + startTime, "5");

        robot.splineMove(toThirdStone, 1, 1, 0.75, 20, 0, Math.toRadians(270), angleLockThird,
                toThirdStoneActions, true, 4500);

        robot.dumpPoints("" + startTime, "6");

        robot.splineMove(toDepositThirdStone, 1, 1, .4, 35, Math.toRadians(180), Math.toRadians(270), 90, toParkAfterThirdStoneActions, true, 5000, true, new Point(20, 5));

        robot.getBackClamp().setPosition(robot.BACKCLAMP_RELEASED);

        robot.getLinearOpMode().sleep(250); // Wait to finish releasing foundation

        robot.dumpPoints("" + startTime, "7");

        robot.splineMove(toPark, .65, 1, 0.4, 10, 0, Math.toRadians(270), 35, toParkActions, true, 5000);

        robot.dumpPoints("" + startTime, "8");

        robot.brakeRobot();
    }
}
