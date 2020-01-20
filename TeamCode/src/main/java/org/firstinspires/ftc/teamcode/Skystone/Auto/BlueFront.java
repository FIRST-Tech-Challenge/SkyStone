package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Action;
import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Enums.ActionType;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

import java.util.ArrayList;

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
        double secondSkyStoneY = 9;
        double secondSkyStoneX = 50;
        double thirdStoneY = 25;
        double thirdStoneX = 53;
        double anglelock = 37;
        double angleLockAngle = Math.toRadians(70);

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
            secondSkyStoneY = -2;
            secondSkyStoneX = 49;
            anglelock = 34;
            thirdStoneX = 53;
            thirdStoneY = 18;
        } else if (skystoneLocation == Vision.Location.RIGHT) {
            firstSkystoneY = 4;
            secondSkyStoneY = 15;
            secondSkyStoneX = 49;
            anglelock = 45;
            thirdStoneX = 75;
            thirdStoneY = 31.5;
            angleLockAngle = Math.toRadians(65);
        }

        double[][] toFirstStone = {
                {0, 0, 10, 0},
                {10, firstSkystoneY, 10, 0},
                {48, firstSkystoneY, 10, 0}};
        ArrayList<Action> toFirstStoneActions = new ArrayList<>();
        toFirstStoneActions.add(new Action(ActionType.START_INTAKE, new Point(0,0), robot));


        double[][] toFoundation = {
                toFirstStone[toFirstStone.length - 1],
                {33, firstSkystoneY-5, 0, -10},
                {31, -17, -10, -20},
                {27, -20, -10, -20},
                {27, -30, -10, -20},
                {27, -43, -10, -20},
                {26, -55, 0, -20},
                {26, -67, 0, -20},
                {27, -68, 0, -20},
                {29, -74, 0, -20},
                {36.5, -83, 0, -10}};
        ArrayList<Action> toFoundationActions = new ArrayList<>();
        toFoundationActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(24,-26), robot));
        toFoundationActions.add(new Action(ActionType.EXTEND_FOUNDATION, robot, true));
        toFoundationActions.add(new Action(ActionType.STOP_INTAKE, new Point(24,-45), robot));

        double[][] toReleaseFoundation = {
                {toFoundation[toFoundation.length - 1][0], toFoundation[toFoundation.length - 1][1], -10, 0},
                {12, -74, 10, 0},
                {8, -69, 10, 0},
        };
        ArrayList<Action> toReleaseFoundationActions = new ArrayList<Action>();
        toReleaseFoundationActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point (27,-73), robot));
        toReleaseFoundationActions.add(new Action(ActionType.RELEASE_FOUNDATION, robot, true));

        double[][] toSecondStone = {
                {toFoundation[toFoundation.length - 1][0], toFoundation[toFoundation.length - 1][1], -10, 0},
                {24, -61, -10, 0},
                {24, -29, 0, 10},
                {24, secondSkyStoneY - 10, 0, -10},
                {secondSkyStoneX, secondSkyStoneY, 30, 0},
                {secondSkyStoneX-8, secondSkyStoneY+12, 30, 0}};
        ArrayList<Action> toSecondStoneActions = new ArrayList<>();
        toSecondStoneActions.add(new Action(ActionType.START_INTAKE, new Point(28,10), robot));


        double[][] toDepositSecondStone = {
                {toSecondStone[toSecondStone.length - 1][0], toSecondStone[toSecondStone.length - 1][1], -10, 0},
                {secondSkyStoneX - 8, secondSkyStoneY - 8, -10, 0},
                {secondSkyStoneX - 12, secondSkyStoneY - 10, -10, 0},
                {34, -2, 0, -20},
                {33, -29, 0, -20},
                {30, -63, 0, -10},
                {30, -64, 0, -10},
                {24, -67, 0, -10}};
        ArrayList<Action> toDepositSecondStoneActions = new ArrayList<>();
        toDepositSecondStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(32,-12), robot));
        toDepositSecondStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(35,-15), robot));

        final double[][] toThirdStone = {
                toDepositSecondStone[toDepositSecondStone.length - 1],
                {36, -58, 0, 10},
                {31, -52, 5, -10},
                {31, -49, 0, 10},
                {31, -30, 0, 10},
                {32, -10, 0, -10},
                {43, -6, 0, 10},
                {thirdStoneX, thirdStoneY, 10, 0}};
        ArrayList<Action> toThirdStoneActions = new ArrayList<>();
        toThirdStoneActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(34,-58), robot));
        toThirdStoneActions.add(new Action(ActionType.START_INTAKE, new Point(28,-30), robot));


        double[][] toDepositThirdStone = {
                toThirdStone[toThirdStone.length - 1],
                {36, 15, 0, 10},
                {35, -29, 0, -20},
                {35, -61, 0, -10},
                {36, -74, 0, -10}};
        ArrayList<Action> toParkAfterThirdStoneActions = new ArrayList<>();
        toParkAfterThirdStoneActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(35,-12), robot));
        toParkAfterThirdStoneActions.add(new Action(ActionType.STOP_INTAKE, new Point(toThirdStone[toThirdStone.length - 1][0] - 5, toThirdStone[toThirdStone.length - 1][1] - 20), robot));
        toParkAfterThirdStoneActions.add(new Action(ActionType.START_INTAKE, new Point(21,-20), robot));
        
        double[][] toDepositThirdStoneOtherwise = {
                toThirdStone[toThirdStone.length - 1],
                {36, 15, 0, 10},
                {35, -29, 0, -20},
                {35, -61, 0, -10},
                {36, -74, 0, -10}};
        ArrayList<Action> toParkAfterThirdStoneActionsOtherwise = new ArrayList<>();
        toParkAfterThirdStoneActionsOtherwise.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(23,-5), robot));
        toParkAfterThirdStoneActionsOtherwise.add(new Action(ActionType.STOP_INTAKE, new Point(toThirdStone[toThirdStone.length - 1][0] - 5, toThirdStone[toThirdStone.length - 1][1] - 20), robot));
        toParkAfterThirdStoneActionsOtherwise.add(new Action(ActionType.START_INTAKE, new Point(21,-20), robot));

        double[][] toPark = {
                {toDepositThirdStone[toDepositThirdStone.length - 1][0], toDepositThirdStone[toDepositThirdStone.length - 1][1], 0, 10},
                {36, -55, 0, 10},
                {37, -36, 0, 10}};

        ArrayList<Action> toParkActions = new ArrayList<>();
        toParkActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(37,-70), robot));

        double[][] toParkDitch = {
                {toThirdStone[toThirdStone.length - 1][0], toThirdStone[toThirdStone.length - 1][1], -10, -10},
                {37, toThirdStone[toThirdStone.length - 1][1] - 20, -10, -10},
                {37, -30, -10, -10}};
        ArrayList<Action> toParkDitchActions = new ArrayList<>();
        toParkDitchActions.add(new Action(ActionType.DROPSTONE_AND_RETRACT_OUTTAKE, new Point(37,-70), robot));

        intake(true);
        robot.splineMove(toFirstStone, 0.6, 1, 0.55, 35, 0, 0, 20,
                toFirstStoneActions, true, 2500);
        //to first stone is 1
        robot.dumpPoints("" + startTime, "1");

        robot.splineMove(toFoundation, 1, 1, 0.6, 25, Math.toRadians(180), Math.toRadians(180), 25,
                toFoundationActions, true, 3500);
        // to foundation is 2
        robot.dumpPoints("" + startTime, "2");

        // get ready to pull foundation
        robot.getLinearOpMode().sleep(150); // Allow foundation movers to deploy

        robot.splineMove(toReleaseFoundation, 1, 1, .8, 5, 0, Math.toRadians(90), 11,
                toReleaseFoundationActions, true, 2250);

        robot.splineMove(toSecondStone, 1, 1, 0.95, 25, 0, angleLockAngle, anglelock,
                toSecondStoneActions, true, 4650);
        //to second stone is 3
        robot.dumpPoints("" + startTime, "3");

        robot.splineMove(toDepositSecondStone, 1, 1, 0.5, 40, Math.toRadians(180), Math.toRadians(90), 18,
                toDepositSecondStoneActions, true, 3250);

        //to deposit second stone is 4
        robot.dumpPoints("" + startTime, "4");

        robot.foundationMovers(false);
        robot.getBackClamp().setPosition(robot.FRONTCLAMP_RELEASED);
        robot.brakeRobot();

        robot.splineMove(toThirdStone, 0.5, 1, 1, 70, 0, Math.toRadians(90), 20,
                toThirdStoneActions, true, 4500);
        //to thrid stone is 5
        robot.dumpPoints("" + startTime, "5");
        if (SystemClock.elapsedRealtime() - startTime < 26000) {
            if(skystoneLocation == Vision.Location.RIGHT) {
                robot.splineMove(toDepositThirdStone, 1, 1, 0.5, 40, Math.toRadians(180), Math.toRadians(90), 20, toParkAfterThirdStoneActions, true, 4000);
            }else{
                robot.splineMove(toDepositThirdStoneOtherwise, 1, 1, 0.5, 40, Math.toRadians(180), Math.toRadians(90), 20, toParkAfterThirdStoneActionsOtherwise, true, 4000);
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