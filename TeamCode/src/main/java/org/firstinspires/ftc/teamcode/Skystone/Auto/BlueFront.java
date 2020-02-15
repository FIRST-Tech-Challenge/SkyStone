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
        double secondSkyStoneY = 8;
        double secondSkyStoneX = 42.5;
        double thirdStoneY = 23;
        double thirdStoneX = 54;
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
            firstSkystoneY = -8;
            secondSkyStoneY = 1;
            secondSkyStoneX = 48;
            anglelock = 30;
            thirdStoneX = 53;
            thirdStoneY = 14;
            angleLockThird = 70;
            thirdStoneXPath = 54;
            mecanumPoint = -2;
        } else if (skystoneLocation == Vision.Location.RIGHT) {
            firstSkystoneY = 18;
            secondSkyStoneY = 18;
            secondSkyStoneX = 42;
            anglelock = 30;
            thirdStoneX = 66;
            thirdStoneY = 16.25;
        }

        double[][] toFirstStone = {
                {0, 0, 10, 0},
                {48, firstSkystoneY, 10, 0}};
        ArrayList<Action> toFirstStoneActions = new ArrayList<Action>();
        toFirstStoneActions.add(new Action(ActionType.START_INTAKE, new Point(0,0), robot));

        double[][] toFoundation = {
                toFirstStone[toFirstStone.length - 1],
                {36, firstSkystoneY - 2, 0, 10},
                {33, -17, -10, 20},
                {31, -20, -10, 20},
                {31, -30, -10, 20},
                {29, -43, -10, 20},
                {26, -55, 0, 20},
                {25, -60, 0, 20},
                {25, -64, 0, 20},
                {25, -66, 0, 20},
                {26, -71, 0, 20},
                {38.5, -80, 0, 10}};
        ArrayList<Action> toFoundationActions = new ArrayList<Action>();
        toFoundationActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(28,-15), robot, 200));
        toFoundationActions.add(new Action(ActionType.EXTEND_FOUNDATION, robot, true));
        toFoundationActions.add(new Action(ActionType.STOP_INTAKE, new Point(28,0), robot));

        double[][] toReleaseFoundation = {
                {toFoundation[toFoundation.length - 1][0], toFoundation[toFoundation.length - 1][1], -10, 0},
                {27, -70, 10, 0},
                {19, -66, 10, 0},
        };
        ArrayList<Action> toReleaseFoundationActions = new ArrayList<Action>();
        toReleaseFoundationActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point (33,-73), robot, 0));
        toReleaseFoundationActions.add(new Action(ActionType.RELEASE_FOUNDATION, robot, true));

        double[][] toSecondStone = {
                {toReleaseFoundation[toReleaseFoundation.length - 1][0], toReleaseFoundation[toReleaseFoundation.length - 1][1], -10, 0},
                {28, -60, -10, 0},
                {29, -29, 0, -10},
                {35, secondSkyStoneY - 5, 0, 10},
                {51, secondSkyStoneY, 30, 0},
                {48, secondSkyStoneY+6, 30, 0}};
        ArrayList<Action> toSecondStoneActions = new ArrayList<Action>();
        toSecondStoneActions.add(new Action(ActionType.START_INTAKE, new Point(33,10), robot));

        double[][] toDepositSecondStone = {
                toSecondStone[toSecondStone.length-1],
                {35, secondSkyStoneY-10, 0, 20},
                {36, -29, 0, 20},
                {36, -47, 0, 10},
                {36, -50, 0, 10},
                {37, -78, 0, 10}};
        ArrayList<Action> toDepositSecondStoneActions = new ArrayList<Action>();
        toDepositSecondStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(35,-5), robot, 400));
        toDepositSecondStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(35,7), robot));

        final double[][] toThirdStone = {
                toDepositSecondStone[toDepositSecondStone.length - 1],
                {37, -45, 5, 10},
                {37, -40, 0, -10},
                {41, -30, 0, -10},
                {48, -15, 0, -10},
                {thirdStoneXPath, 5, 0, -10},
                {thirdStoneXPath+9, 0, 0, -10},
                {thirdStoneX, thirdStoneY-10, 10, 0},
                {thirdStoneX, thirdStoneY, 10, 0}};
        ArrayList<Action> toThirdStoneActions = new ArrayList<Action>();
        toThirdStoneActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(36,-71), robot, 0));
        toThirdStoneActions.add(new Action(ActionType.START_INTAKE, new Point(43,0), robot));

        double[][] toDepositThirdStone = {
                toThirdStone[toThirdStone.length-1],
                {42, 5, 0, -10},
                {39, -5, 0, 20},
                {39, -15, 0, 20},
                {39, -20, 0, 20},
                {38, -29, 0, 20},
                {38, -35, 0, 20},
                {38, -45, 0, 10},
                {38, -55, 0, 20},
                {38, -65, 0, 20},
                {38, -78, 0, 10}};
        ArrayList<Action> toParkAfterThirdStoneActions = new ArrayList<Action>();
        toParkAfterThirdStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(40,-5), robot, 450));
        toParkAfterThirdStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(toThirdStone[toThirdStone.length - 1][0] - 15, toThirdStone[toThirdStone.length - 1][1] - 15), robot));

        double[][] toPark = {
                {toDepositThirdStone[toDepositThirdStone.length - 1][0], toDepositThirdStone[toDepositThirdStone.length - 1][1], 0, -10},
                {41, -55, 0, -10},
                {41, -34, 0, -10}};
        ArrayList<Action> toParkActions = new ArrayList<Action>();
        toParkActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(35,-74), robot, 0));

        double[][] toParkDitch = {
                {toThirdStone[toThirdStone.length - 1][0], toThirdStone[toThirdStone.length - 1][1], -10, 10},
                {18, toThirdStone[toThirdStone.length - 1][1] + 20, -10, 10},
                {18, 30, -10, 10}};
        ArrayList<Action> toParkDitchActions = new ArrayList<Action>();
        toParkDitchActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(25,65), robot, 0));

        robot.splineMove(toFirstStone, 0.65, 0.5, 0.55, 35, 0, Math.toRadians(0), 0,
                toFirstStoneActions, true, 2500);

        robot.dumpPoints("" + startTime, "1");

        robot.splineMove(toFoundation, 1, 1, 1, 20, Math.toRadians(180), Math.toRadians(180), 30,
                toFoundationActions, true, 4000, true, new Point(20,0));

        robot.foundationMovers(true);

        robot.getLinearOpMode().sleep(150); // Allow foundation movers to deploy

        robot.dumpPoints("" + startTime, "2");

        robot.splineMove(toReleaseFoundation, 1, 1, 1, 5, 0, Math.toRadians(90), 12,
                toReleaseFoundationActions, true, 2250);

        robot.foundationMovers(false);

        robot.getLinearOpMode().sleep(150); // Allow foundation movers to deploy

        robot.dumpPoints("" + startTime, "3");

//        robot.getLinearOpMode().sleep(150); // Wait to finish releasing foundation

        robot.splineMove(toSecondStone, 1, 1, 0.6, 35, 0, Math.toRadians(70), anglelock,
                toSecondStoneActions, true, 5000);

        robot.dumpPoints("" + startTime, "4");

        robot.splineMove(toDepositSecondStone, 1, 1, 0.5, 35, Math.toRadians(180), Math.toRadians(90), 95,
                toDepositSecondStoneActions, true, 4000, true, new Point(35,mecanumPoint));

        robot.getBackClamp().setPosition(robot.BACKCLAMP_RELEASED);
        robot.getFrontClamp().setPosition(robot.FRONTCLAMP_RELEASED);

        robot.getLinearOpMode().sleep(250); // Wait to finish releasing foundation

        robot.dumpPoints("" + startTime, "5");

        robot.splineMove(toThirdStone, 1, 1, 0.75, 20, 0, Math.toRadians(90), angleLockThird,
                toThirdStoneActions, true, 4250);

        robot.dumpPoints("" + startTime, "6");

        robot.splineMove(toDepositThirdStone, 1, 1, .5, 35, Math.toRadians(180), Math.toRadians(90), 95, toParkAfterThirdStoneActions, true, 4000, true, new Point(41,mecanumPoint+10));

        robot.getBackClamp().setPosition(robot.BACKCLAMP_RELEASED);
        robot.getFrontClamp().setPosition(robot.FRONTCLAMP_RELEASED);

        robot.getLinearOpMode().sleep(250); // Wait to finish releasing foundation

        robot.dumpPoints("" + startTime, "7");

        robot.splineMove(toPark, .65, 1, 0.4, 10, 0, Math.toRadians(90), 35, toParkActions, true, 3000);

        robot.dumpPoints("" + startTime, "8");
//        if (SystemClock.elapsedRealtime() - startTime < 29000) {
//            robot.splineMove(toDepositThirdStone, 1, 1, .44, 30, Math.toRadians(180), Math.toRadians(270), 65, toParkAfterThirdStoneActions, true, 3500);
//
//            robot.dumpPoints("" + startTime, "7");
//
//            robot.splineMove(toPark, .6, 1, 0.3, 10, 0, Math.toRadians(270), 5, toParkActions, false, 5000);
//
//            robot.dumpPoints("" + startTime, "8");
//        } else {
//            robot.splineMove(toParkDitch, .7, 1, 0.55, 17, Math.toRadians(180), Math.toRadians(270), 5, toParkDitchActions, false, 5000);
//
//            robot.dumpPoints("" + startTime, "8");
//        }
    }
}