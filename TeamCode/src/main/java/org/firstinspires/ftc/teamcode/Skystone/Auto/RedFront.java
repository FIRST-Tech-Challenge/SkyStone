package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Action;
import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Enums.ActionType;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

import java.util.ArrayList;

@Autonomous(name = "RedFrontOB", group = "LinearOpmode")
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
            skystoneLocation = vision.runDetection(false, true);
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
            secondSkyStoneX = 37;
            anglelock = 30;
            thirdStoneX = 59;
            thirdStoneY = -30;
        } else if (skystoneLocation == Vision.Location.RIGHT) {
            firstSkystoneY = 4.5;
            secondSkyStoneY = -8;
            secondSkyStoneX = 42;
            anglelock = 33;
            thirdStoneX = 33;
            thirdStoneY = -19;
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
                {26, 17, -10, 20},
                {24, 20, -10, 20},
                {24, 30, -10, 20},
                {23, 43, -10, 20},
                {23, 55, 0, 20},
                {22, 67, 0, 20},
                {22, 68, 0, 20},
                {21, 72, 0, 20},
                {21, 75, 0, 20},
                {32, 79, 0, 10}};
        ArrayList<Action> toFoundationActions = new ArrayList<Action>();
        toFoundationActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(24,35), robot, 150));
        toFoundationActions.add(new Action(ActionType.EXTEND_FOUNDATION, robot, true));
        toFoundationActions.add(new Action(ActionType.STOP_INTAKE, new Point(24,45), robot));

        double[][] toReleaseFoundation = {
                {toFoundation[toFoundation.length - 1][0], toFoundation[toFoundation.length - 1][1], -10, 0},
                {20, 70, 10, 0},
                {10, 67, 10, 0},
        };
        ArrayList<Action> toReleaseFoundationActions = new ArrayList<Action>();
        toReleaseFoundationActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point (22,73), robot, 0));
        toReleaseFoundationActions.add(new Action(ActionType.RELEASE_FOUNDATION, robot, true));

        double[][] toSecondStone = {
                {toReleaseFoundation[toReleaseFoundation.length - 1][0], toReleaseFoundation[toReleaseFoundation.length - 1][1], -10, 0},
                {20, 60, -10, 0},
                {17, 29, 0, -10},
                {17, secondSkyStoneY + 5, 0, 10},
                {secondSkyStoneX, secondSkyStoneY, 30, 0},
                {secondSkyStoneX-8, secondSkyStoneY-10, 30, 0}};
        ArrayList<Action> toSecondStoneActions = new ArrayList<Action>();
        toSecondStoneActions.add(new Action(ActionType.START_INTAKE, new Point(28,-10), robot));

        double[][] toDepositSecondStone = {
                {19, 0, 0, 20},
                {19, 29, 0, 20},
                {19, 47, 0, 10},
                {19, 50, 0, 10},
                {19, 74, 0, 10}};
        ArrayList<Action> toDepositSecondStoneActions = new ArrayList<Action>();
        toDepositSecondStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(28,17), robot, 150));
        toDepositSecondStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(35,15), robot));
        toDepositSecondStoneActions.add(new Action(ActionType.START_INTAKE, new Point(35,25), robot));

        final double[][] toThirdStone = {
                toDepositSecondStone[toDepositSecondStone.length - 1],
                {20, 45, 5, 10},
                {19, 40, 0, -10},
                {19, 30, 0, -10},
                {19, 10, 0, -10},
                {32, 6, 0, -10},
                {thirdStoneX, thirdStoneY, 10, 0}};
        ArrayList<Action> toThirdStoneActions = new ArrayList<Action>();
        toThirdStoneActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(22,58), robot, 0));
        toThirdStoneActions.add(new Action(ActionType.START_INTAKE, new Point(28,30), robot));

        double[][] toDepositThirdStone = {
                {14, -12, 0, -10},
                {18, -5, 0, -10},
                {18, 5, 0, 20},
                {18, 15, 0, 20},
                {18, 20, 0, 20},
                {18, 29, 0, 20},
                {18, 35, 0, 20},
                {18, 45, 0, 10},
                {18, 55, 0, 20},
                {18, 65, 0, 20},
                {18, 71, 0, 10}};
        ArrayList<Action> toParkAfterThirdStoneActions = new ArrayList<Action>();
        toParkAfterThirdStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(23,7), robot, 150));
        toParkAfterThirdStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(toThirdStone[toThirdStone.length - 1][0] - 15, toThirdStone[toThirdStone.length - 1][1] + 20), robot, 150));
        toParkAfterThirdStoneActions.add(new Action(ActionType.START_INTAKE, new Point(21,20), robot));

        double[][] toPark = {
                {toDepositThirdStone[toDepositThirdStone.length - 1][0], toDepositThirdStone[toDepositThirdStone.length - 1][1], 0, -10},
                {16, 55, 0, -10},
                {17, 34, 0, -10}};
        ArrayList<Action> toParkActions = new ArrayList<Action>();
        toParkActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(25,65), robot, 0));

        double[][] toParkDitch = {
                {toThirdStone[toThirdStone.length - 1][0], toThirdStone[toThirdStone.length - 1][1], -10, 10},
                {18, toThirdStone[toThirdStone.length - 1][1] + 20, -10, 10},
                {18, 30, -10, 10}};
        ArrayList<Action> toParkDitchActions = new ArrayList<Action>();
        toParkDitchActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(25,65), robot, 0));

        robot.splineMove2(toFirstStone, 0.6, 1, 0.55, 35, 0, 0, 20,
                toFirstStoneActions, true, 3000);

        robot.dumpPoints("" + startTime, "1");

        robot.splineMove2(toFoundation, 1, 1, 0.65, 20, Math.toRadians(180), Math.toRadians(180), 20,
                toFoundationActions, true, 3500);

        robot.getLinearOpMode().sleep(150); // Allow foundation movers to deploy

        robot.dumpPoints("" + startTime, "2");

        robot.splineMove2(toReleaseFoundation, 1, 1, .8, 5, 0, Math.toRadians(270), 10,
                toReleaseFoundationActions, true, 3500);

        robot.dumpPoints("" + startTime, "3");

//        robot.getLinearOpMode().sleep(150); // Wait to finish releasing foundation

        robot.splineMove2(toSecondStone, 1, 1, 0.7, 25, 0, Math.toRadians(297), anglelock,
                toSecondStoneActions, true, 4000);

        robot.dumpPoints("" + startTime, "4");

        robot.splineMove2(toDepositSecondStone, 1, 1, 0.44, 30, Math.toRadians(180), Math.toRadians(270), 30,
                toDepositSecondStoneActions, true, 3500);

        robot.dumpPoints("" + startTime, "5");

        robot.splineMove2(toThirdStone, 1, 1, 1, 70, 0, Math.toRadians(270), 35,
                toThirdStoneActions, true, 4000);

        robot.dumpPoints("" + startTime, "6");
        if (SystemClock.elapsedRealtime() - startTime < 29000) {
            robot.splineMove2(toDepositThirdStone, 1, 1, .44, 30, Math.toRadians(180), Math.toRadians(270), 35, toParkAfterThirdStoneActions, true, 3500);

            robot.dumpPoints("" + startTime, "7");

            robot.splineMove2(toPark, .6, 1, 0.3, 10, 0, Math.toRadians(270), 5, toParkActions, false, 5000);

            robot.dumpPoints("" + startTime, "8");
        } else {
            robot.splineMove2(toParkDitch, .7, 1, 0.55, 17, Math.toRadians(180), Math.toRadians(270), 5, toParkDitchActions, false, 5000);

            robot.dumpPoints("" + startTime, "8");
        }
    }
}
