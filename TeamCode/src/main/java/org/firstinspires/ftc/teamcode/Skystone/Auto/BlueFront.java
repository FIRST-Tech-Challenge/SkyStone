package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Action;
import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Enums.ActionType;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

import java.util.ArrayList;

@Autonomous(name = "BlueFrontAS", group = "LinearOpmode")
public class BlueFront extends AutoBase {
    @Override
    public void runOpMode() {
        long startTime;
        initLogic();

        waitForStart();
        startTime = SystemClock.elapsedRealtime();

        // Positions assuming center Skystone
        double firstSkystoneY = 7;
        double secondSkyStoneY = 9;
        double thirdStoneY = 26;
        double thirdStoneX = 56;
        double anglelock = 35;
        double angleLockThird = 75;
        double thirdStoneXPath = 49;
        double mecanumPoint = 5;

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
            firstSkystoneY = -10;
            secondSkyStoneY = -1;
            anglelock = 30;
            thirdStoneX = 55;
            thirdStoneY = 13.5;
            angleLockThird = 70;
            thirdStoneXPath = 54;
            mecanumPoint = -2;
        } else if (skystoneLocation == Vision.Location.RIGHT) {
            firstSkystoneY = 16;
            secondSkyStoneY = 18;
            anglelock = 30;
            thirdStoneX = 68;
            thirdStoneY = 19;
        }

        double[][] toFirstStone = {
                {0, 0},
                {48, firstSkystoneY}};
        ArrayList<Action> toFirstStoneActions = new ArrayList<Action>();
        toFirstStoneActions.add(new Action(ActionType.START_INTAKE, new Point(0, 0), robot));

        double[][] toFoundation = {
                toFirstStone[toFirstStone.length - 1],
                {36, firstSkystoneY - 2},
                {33, -17},
                {31, -20},
                {31, -30},
                {29, -43},
                {26, -55},
                {25, -60},
                {25, -64},
                {25, -66},
                {26, -71},
                {38.5, -84}};
        ArrayList<Action> toFoundationActions = new ArrayList<Action>();
        toFoundationActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(28, -15), robot, 50));
        toFoundationActions.add(new Action(ActionType.EXTEND_FOUNDATION, robot, true));
        toFoundationActions.add(new Action(ActionType.STOP_INTAKE, new Point(28, -7), robot));

        double[][] toReleaseFoundation = {
                {toFoundation[toFoundation.length - 1][0], toFoundation[toFoundation.length - 1][1], -10, 0},
                {27, -70},
                {19, -66},
        };
        ArrayList<Action> toReleaseFoundationActions = new ArrayList<Action>();
        toReleaseFoundationActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(33, -73), robot, 0));
        toReleaseFoundationActions.add(new Action(ActionType.RELEASE_FOUNDATION, robot, true));

        double[][] toSecondStone = {
                {toReleaseFoundation[toReleaseFoundation.length - 1][0], toReleaseFoundation[toReleaseFoundation.length - 1][1], -10, 0},
                {28, -60},
                {29, -29},
                {37, secondSkyStoneY - 5},
                {54, secondSkyStoneY},
                {51, secondSkyStoneY + 6}};
        ArrayList<Action> toSecondStoneActions = new ArrayList<Action>();
        toSecondStoneActions.add(new Action(ActionType.START_INTAKE, new Point(35, 0), robot));

        double[][] toDepositSecondStone = {
                toSecondStone[toSecondStone.length - 1],
                {34, secondSkyStoneY - 10},
                {35, -29},
                {36, -47},
                {35, -50},
                {33, -78}};
        ArrayList<Action> toDepositSecondStoneActions = new ArrayList<Action>();
        toDepositSecondStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(35, -5), robot, 400));
        toDepositSecondStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(35, 0), robot));

        double[][] toThirdStone = {
                toDepositSecondStone[toDepositSecondStone.length - 1],
                {36, -45},
                {36, -40},
                {36, -30},
                {48, -15},
                {thirdStoneXPath, 5},
                {thirdStoneXPath + 9, 0},
                {thirdStoneX, thirdStoneY - 10},
                {thirdStoneX, thirdStoneY}};
        ArrayList<Action> toThirdStoneActions = new ArrayList<Action>();
        toThirdStoneActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(36, -69), robot, 0));
        toThirdStoneActions.add(new Action(ActionType.START_INTAKE, new Point(43, 0), robot));

        double[][] toDepositThirdStone = {
                toThirdStone[toThirdStone.length - 1],
                {38, 5},
                {38, -5},
                {38, -15},
                {38, -20},
                {38, -29},
                {38, -35},
                {38, -45},
                {39, -55},
                {39, -65},
                {37, -78}};
        ArrayList<Action> toParkAfterThirdStoneActions = new ArrayList<Action>();
        toParkAfterThirdStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(40, 0), robot, 400));
        toParkAfterThirdStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(toThirdStone[toThirdStone.length - 1][0] - 15, toThirdStone[toThirdStone.length - 1][1] - 15), robot));

        double[][] toPark = {
                {toDepositThirdStone[toDepositThirdStone.length - 1][0], toDepositThirdStone[toDepositThirdStone.length - 1][1], 0, -10},
                {38, -55},
                {38, -34}};
        ArrayList<Action> toParkActions = new ArrayList<Action>();
        toParkActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(35, -74), robot, 0));

        robot.splineMove(toFirstStone, 0.65, 0.5, .7, 35, 0, Math.toRadians(0), 0,
                toFirstStoneActions, true, 2500);

        robot.dumpPoints("" + startTime, "1");

        robot.splineMove(toFoundation, 1, 1, 0.4, 20, Math.toRadians(180), Math.toRadians(180), 25,
                toFoundationActions, true, 4000, true, new Point(20, 0));

        robot.foundationMovers(true);

        robot.getLinearOpMode().sleep(150); // Allow foundation movers to deploy

        robot.dumpPoints("" + startTime, "2");

        robot.splineMove(toReleaseFoundation, 1, 1, 1, 5, 0, Math.toRadians(90), 12,
                toReleaseFoundationActions, true, 2250);

        robot.foundationMovers(false);

        robot.getLinearOpMode().sleep(150); // Allow foundation movers to deploy

        robot.dumpPoints("" + startTime, "3");

        robot.splineMove(toSecondStone, 1, 1, 0.6, 35, 0, Math.toRadians(70), anglelock,
                toSecondStoneActions, true, 5000);

        robot.dumpPoints("" + startTime, "4");

        robot.splineMove(toDepositSecondStone, 1, 1, 0.4, 35, Math.toRadians(180), Math.toRadians(90), 80,
                toDepositSecondStoneActions, true, 4250, true, new Point(35, mecanumPoint));

        robot.getBackClamp().setPosition(robot.BACKCLAMP_RELEASED);
        robot.getFrontClamp().setPosition(robot.FRONTCLAMP_RELEASED);

        robot.getLinearOpMode().sleep(250); // Wait to finish releasing foundation

        robot.dumpPoints("" + startTime, "5");

        robot.splineMove(toThirdStone, 1, 1, 0.65, 20, 0, Math.toRadians(90), angleLockThird,
                toThirdStoneActions, true, 4250);

        robot.dumpPoints("" + startTime, "6");

        robot.splineMove(toDepositThirdStone, 1, 1, .4, 35, Math.toRadians(180), Math.toRadians(90), 80, toParkAfterThirdStoneActions, true, 4500, true, new Point(41, mecanumPoint + 10));

        robot.getBackClamp().setPosition(robot.BACKCLAMP_RELEASED);
        robot.getFrontClamp().setPosition(robot.FRONTCLAMP_RELEASED);

        robot.getLinearOpMode().sleep(250); // Wait to finish releasing foundation

        robot.dumpPoints("" + startTime, "7");

        robot.splineMove(toPark, .65, 1, 0.4, 10, 0, Math.toRadians(90), 35, toParkActions, true, 3000);

        robot.dumpPoints("" + startTime, "8");
    }
}