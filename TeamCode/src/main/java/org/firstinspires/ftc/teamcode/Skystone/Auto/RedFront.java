package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Action;
import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Enums.ActionType;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

import java.util.ArrayList;

@Autonomous(name = "RedFront", group = "LinearOpmode")
public class RedFront extends AutoBase {
    @Override
    public void runOpMode() {
        long startTime;
        initLogic();

        waitForStart();
        startTime = SystemClock.elapsedRealtime();

        // Positions assuming center Skystone
        double secondSkystoneY = 0;
        double firstSkyStoneY = -15;
        double thirdStoneY = -27;
        double thirdStoneX = 70;
        double anglelock = 33;

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
            secondSkystoneY = -12;
            firstSkyStoneY = -24;
            anglelock = 30;
            thirdStoneX = 67;
            thirdStoneY = -35;
        } else if (skystoneLocation == Vision.Location.RIGHT) {
            secondSkystoneY = 10;
            firstSkyStoneY = -17;
            anglelock = 35;
            thirdStoneX = 43;
            thirdStoneY = -19;
        }

        double[][] toFirstStone = {
                {0, 0, 10, 0},
                {10, -8, 10, 0},
                {65, firstSkyStoneY, 10, 0},
                {69, firstSkyStoneY - 6, 10, 0}};
        ArrayList<Action> toFirstStoneActions = new ArrayList<Action>();
        toFirstStoneActions.add(new Action(ActionType.START_INTAKE, new Point(0,0), robot));

        double[][] toFoundation = {
                toFirstStone[toFirstStone.length - 1],
                {34, firstSkyStoneY + 5, 0, 10},
                {25, 17, -10, 20},
                {27, 20, -10, 20},
                {27, 30, -10, 20},
                {27, 43, -10, 20},
                {27, 55, 0, 20},
                {25, 60, 0, 20},
                {24, 64, 0, 20},
                {24, 66, 0, 20},
                {24, 71, 0, 20},
                {33, 79, 0, 10}};
        ArrayList<Action> toFoundationActions = new ArrayList<Action>();
        toFoundationActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(20,15), robot, 200));
        toFoundationActions.add(new Action(ActionType.EXTEND_FOUNDATION, robot, true));
        toFoundationActions.add(new Action(ActionType.STOP_INTAKE, new Point(20,15), robot));

        double[][] toReleaseFoundation = {
                {toFoundation[toFoundation.length - 1][0], toFoundation[toFoundation.length - 1][1], -10, 0},
                {20, 70, 10, 0},
                {12, 66, 10, 0},
        };
        ArrayList<Action> toReleaseFoundationActions = new ArrayList<Action>();
        toReleaseFoundationActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point (22,73), robot, 0));
        toReleaseFoundationActions.add(new Action(ActionType.RELEASE_FOUNDATION, robot, true));

        double[][] toSecondStone = {
                {toReleaseFoundation[toReleaseFoundation.length - 1][0], toReleaseFoundation[toReleaseFoundation.length - 1][1], -10, 0},
                {22, 60, -10, 0},
                {24, 29, 0, -10},
                {23, 24, 0, 10},
                {48, secondSkystoneY, 30, 0}};
        ArrayList<Action> toSecondStoneActions = new ArrayList<Action>();
        toSecondStoneActions.add(new Action(ActionType.START_INTAKE, new Point(24, 30), robot));

        double[][] toDepositSecondStone = {
                toSecondStone[toSecondStone.length - 1],
                {38, secondSkystoneY, 0, 0},
                {35, 14, 0, 0},
                {25, 16, 0 ,0},
                {24, 47, 0, 10},
                {22, 74, 0, 10}};
        ArrayList<Action> toDepositSecondStoneActions = new ArrayList<Action>();
        toDepositSecondStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(25,9), robot, 200));
        toDepositSecondStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(25,11), robot));

        final double[][] toThirdStone = {
                toDepositSecondStone[toDepositSecondStone.length - 1],
                {24, 45, 5, 10},
                {24, 35, 0, -10},
                {24, 25, 0, -10},
                {36, 10, 0, -10},
                {thirdStoneX, thirdStoneY, 10, 0}};
        ArrayList<Action> toThirdStoneActions = new ArrayList<Action>();
        toThirdStoneActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(24,40), robot, 0));
        toThirdStoneActions.add(new Action(ActionType.START_INTAKE, new Point(24,30), robot));

        double[][] toDepositThirdStone = {
                {24, -5, 0, -10},
                {24, 5, 0, 20},
                {24, 15, 0, 20},
                {24, 20, 0, 20},
                {24, 29, 0, 20},
                {24, 35, 0, 20},
                {24, 45, 0, 10},
                {24, 55, 0, 20},
                {24, 65, 0, 20},
                {26, 75, 0, 10}};
        ArrayList<Action> toParkAfterThirdStoneActions = new ArrayList<Action>();
        toParkAfterThirdStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(20,7), robot, 450));
        toParkAfterThirdStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(toThirdStone[toThirdStone.length - 1][0] - 20, toThirdStone[toThirdStone.length - 1][1] + 25), robot));

        double[][] toPark = {
                {toDepositThirdStone[toDepositThirdStone.length - 1][0], toDepositThirdStone[toDepositThirdStone.length - 1][1], 0, -10},
                {23, 55, 0, -10},
                {23, 34, 0, -10}};
        ArrayList<Action> toParkActions = new ArrayList<Action>();
        toParkActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(25,74), robot, 0));

        double[][] toParkDitch = {
                {toThirdStone[toThirdStone.length - 1][0], toThirdStone[toThirdStone.length - 1][1], -10, 10},
                {18, toThirdStone[toThirdStone.length - 1][1] + 20, -10, 10},
                {18, 30, -10, 10}};
        ArrayList<Action> toParkDitchActions = new ArrayList<Action>();
        toParkDitchActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(25,65), robot, 0));

        robot.splineMove(toFirstStone, 0.6, 1, .7, 35, 0, 0, 20,
                toFirstStoneActions, true, 2500);

        robot.dumpPoints("" + startTime, "1");

        robot.splineMove(toFoundation, 1, 0.7, 0.65, 20, Math.toRadians(180), Math.toRadians(180), 35,
                toFoundationActions, true, 4500);

        robot.foundationMovers(true);

        robot.getLinearOpMode().sleep(150); // Allow foundation movers to deploy

        robot.dumpPoints("" + startTime, "2");

        robot.splineMove(toReleaseFoundation, 1, 1, 1, 5, 0, Math.toRadians(270), 12,
                toReleaseFoundationActions, true, 3500);

        robot.dumpPoints("" + startTime, "3");

//        robot.getLinearOpMode().sleep(150); // Wait to finish releasing foundation

        robot.splineMove(toSecondStone, 1, 1, 0.85, 1, 0, Math.toRadians(320), 3,
                toSecondStoneActions, true, 4000);

        robot.dumpPoints("" + startTime, "4");

        robot.splineMove(toDepositSecondStone, 1, 1, 0.44, 30, Math.toRadians(180), Math.toRadians(270), 65,
                toDepositSecondStoneActions, true, 3000);

        robot.getBackClamp().setPosition(robot.BACKCLAMP_RELEASED);
        robot.getFrontClamp().setPosition(robot.FRONTCLAMP_RELEASED);

        robot.getLinearOpMode().sleep(250);

        robot.dumpPoints("" + startTime, "5");

        robot.splineMove(toThirdStone, 1, 1, 1, 70, 0, Math.toRadians(270), 10,
                toThirdStoneActions, true, 4000);

        robot.dumpPoints("" + startTime, "6");

        robot.splineMove(toDepositThirdStone, 1, 1, .44, 30, Math.toRadians(180), Math.toRadians(270), 75, toParkAfterThirdStoneActions, true, 4500);

        robot.getBackClamp().setPosition(robot.BACKCLAMP_RELEASED);
        robot.getFrontClamp().setPosition(robot.FRONTCLAMP_RELEASED);

        robot.getLinearOpMode().sleep(250); // Wait to finish releasing foundation

        robot.dumpPoints("" + startTime, "7");

        robot.splineMove(toPark, .65, 1, 0.45, 10, 0, Math.toRadians(270), 25, toParkActions, true, 4000);

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
