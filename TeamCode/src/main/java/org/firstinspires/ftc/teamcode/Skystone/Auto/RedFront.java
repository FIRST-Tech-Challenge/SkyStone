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
        double firstSkystoneY = -2;
        double secondSkyStoneY = -15;
        double secondSkyStoneX = 41;
        double thirdStoneY = -27;
        double thirdStoneX = 35.5;
        double anglelock = 33;

        Vision.Location skystoneLocation = Vision.Location.UNKNOWN;
        try {
            skystoneLocation = vision.runDetection(true, true);
        } catch (Exception e) {

        }

        telemetry.addLine("Detection Result: " + skystoneLocation.toString());
        telemetry.update();

        sleep(250);

        position2D.startOdometry();

        // Change Skystone positions if detected left or right
        if (skystoneLocation == Vision.Location.LEFT) {
            firstSkystoneY = -4;
            secondSkyStoneY = -25;
            secondSkyStoneX = 43;
            anglelock = 30;
            thirdStoneX = 45;
            thirdStoneY = -30;
        } else if (skystoneLocation == Vision.Location.RIGHT) {
            firstSkystoneY = 4.5;
            secondSkyStoneY = -8;
            secondSkyStoneX = 42;
            anglelock = 33;
            thirdStoneX = 33;
            thirdStoneY = -23;
        }

        double[][] toFirstStone = {
                {0, 0, 10, 0},
                {10, firstSkystoneY, 10, 0},
                {48, firstSkystoneY, 10, 0}};
        ArrayList<Action> toFirstStoneActions = new ArrayList<Action>();
        toFirstStoneActions.add(new Action(ActionType.START_INTAKE, new Point(0,0), robot));

        double[][] toFoundation = {
                toFirstStone[toFirstStone.length - 1],
                {34, firstSkystoneY + 5, 0, 10},
                {28, 17, -10, 20},
                {24, 20, -10, 20},
                {24, 30, -10, 20},
                {23, 43, -10, 20},
                {22, 55, 0, 20},
                {22, 67, 0, 20},
                {22, 68, 0, 20},
                {21, 74, 0, 20},
                {33, 79, 0, 10}};
        ArrayList<Action> toFoundationActions = new ArrayList<Action>();
        toFoundationActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(24,20), robot));
        toFoundationActions.add(new Action(ActionType.EXTEND_FOUNDATION, robot, true));
        toFoundationActions.add(new Action(ActionType.STOP_INTAKE, new Point(24,45), robot));

        double[][] toReleaseFoundation = {
                {toFoundation[toFoundation.length - 1][0], toFoundation[toFoundation.length - 1][1], -10, 0},
                {9, 70, 10, 0},
                {5, 65, 10, 0},
        };
        ArrayList<Action> toReleaseFoundationActions = new ArrayList<Action>();
        toReleaseFoundationActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point (22,73), robot));
        toReleaseFoundationActions.add(new Action(ActionType.RELEASE_FOUNDATION, robot, true));

        double[][] toSecondStone = {
                {toReleaseFoundation[toReleaseFoundation.length - 1][0], toReleaseFoundation[toReleaseFoundation.length - 1][1], -10, 0},
                {19, 61, -10, 0},
                {19, 29, 0, -10},
                {18, secondSkyStoneY + 5, 0, 10},
                {secondSkyStoneX, secondSkyStoneY, 30, 0},
                {secondSkyStoneX-5, secondSkyStoneY-5, 30, 0}};
        ArrayList<Action> toSecondStoneActions = new ArrayList<Action>();
        toSecondStoneActions.add(new Action(ActionType.START_INTAKE, new Point(28,-10), robot));

        double[][] toDepositSecondStone = {
                {toSecondStone[toSecondStone.length - 1][0], toSecondStone[toSecondStone.length - 1][1], -10, 0},
                {secondSkyStoneX - 7, secondSkyStoneY + 10, -10, 0},
                {secondSkyStoneX - 14, secondSkyStoneY + 8, -10, 0},
                {27, 0, 0, 20},
                {26, 29, 0, 20},
                {19, 64, 0, 10},
                {19, 62, 0, 10},
                {14, 65, 0, 10}};
        ArrayList<Action> toDepositSecondStoneActions = new ArrayList<Action>();
        toDepositSecondStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(28,17), robot));
        toDepositSecondStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(35,15), robot));
        toDepositSecondStoneActions.add(new Action(ActionType.START_INTAKE, new Point(35,25), robot));

        final double[][] toThirdStone = {
                toDepositSecondStone[toDepositSecondStone.length - 1],
                {22, 63, 5, 10},
                {20, 60, 0, -10},
                {20, 30, 0, -10},
                {20, 10, 0, -10},
                {32, 6, 0, -10},
                {thirdStoneX, thirdStoneY, 10, 0}};
        ArrayList<Action> toThirdStoneActions = new ArrayList<Action>();
        toThirdStoneActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(22,58), robot));
        toThirdStoneActions.add(new Action(ActionType.START_INTAKE, new Point(28,30), robot));

        double[][] toDepositThirdStone = {
                toThirdStone[toThirdStone.length - 1],
                {27, -15, 0, -10},
                {22, 29, 0, 20},
                {19, 61, 0, 10},
                {19, 66, 0, 10}};
        ArrayList<Action> toParkAfterThirdStoneActions = new ArrayList<Action>();
        toParkAfterThirdStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(23,7), robot));
        toParkAfterThirdStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(toThirdStone[toThirdStone.length - 1][0] - 15, toThirdStone[toThirdStone.length - 1][1] + 20), robot));
        toParkAfterThirdStoneActions.add(new Action(ActionType.START_INTAKE, new Point(21,20), robot));

        double[][] toPark = {
                {toDepositThirdStone[toDepositThirdStone.length - 1][0], toDepositThirdStone[toDepositThirdStone.length - 1][1], 0, -10},
                {20, 55, 0, -10},
                {19, 34, 0, -10}};
        ArrayList<Action> toParkActions = new ArrayList<Action>();
        toParkActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(25,65), robot));

        double[][] toParkDitch = {
                {toThirdStone[toThirdStone.length - 1][0], toThirdStone[toThirdStone.length - 1][1], -10, 10},
                {26, toThirdStone[toThirdStone.length - 1][1] + 20, -10, 10},
                {26, 30, -10, 10}};
        ArrayList<Action> toParkDitchActions = new ArrayList<Action>();
        toParkDitchActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(25,65), robot));

        robot.splineMove(toFirstStone, 0.6, 1, 0.55, 35, 0, 0, 20,
                toFirstStoneActions, true, 3000);

        robot.dumpPoints("" + startTime, "1");

        robot.splineMove(toFoundation, 1, 1, 0.4, 20, Math.toRadians(180), Math.toRadians(180), 25,
                toFoundationActions, true, 3750);

        robot.getLinearOpMode().sleep(150); // Allow foundation movers to deploy

        robot.dumpPoints("" + startTime, "2");

        robot.splineMove(toReleaseFoundation, 1, 1, .8, 5, 0, Math.toRadians(270), 11,
                toReleaseFoundationActions, true, 2500);

        robot.dumpPoints("" + startTime, "3");

//        robot.getLinearOpMode().sleep(150); // Wait to finish releasing foundation

        robot.splineMove(toSecondStone, 1, 1, 0.6, 25, 0, Math.toRadians(297), anglelock,
                toSecondStoneActions, true, 4500);

        robot.dumpPoints("" + startTime, "4");

        robot.splineMove(toDepositSecondStone, 1, 1, 0.8, 30, Math.toRadians(180), Math.toRadians(270), 18,
                toDepositSecondStoneActions, true, 3500);

        robot.dumpPoints("" + startTime, "5");

        robot.splineMove(toThirdStone, 0.5, 1, 1, 70, 0, Math.toRadians(270), 20,
                toThirdStoneActions, true, 4750);

        robot.dumpPoints("" + startTime, "6");
        if (SystemClock.elapsedRealtime() - startTime < 26000) {
            robot.splineMove(toDepositThirdStone, 1, 1, .5, 30, Math.toRadians(180), Math.toRadians(270), 20, toParkAfterThirdStoneActions, true, 3500);

            robot.dumpPoints("" + startTime, "7");

            robot.splineMove(toPark, .6, 1, 0.3, 10, 0, Math.toRadians(270), 5, toParkActions, false, 5000);

            robot.dumpPoints("" + startTime, "8");
        } else {
            robot.splineMove(toParkDitch, .7, 1, 0.55, 17, Math.toRadians(180), Math.toRadians(270), 5, toParkDitchActions, false, 5000);

            robot.dumpPoints("" + startTime, "8");
        }
    }
}
