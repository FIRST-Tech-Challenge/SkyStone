package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Robot;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

import java.util.HashMap;

@Autonomous(name="BlueFront3", group ="LinearOpmode")
public class BlueFront extends AutoBase{
    @Override
    public void runOpMode() {
        long startTime;
        initLogic();

        waitForStart();
        startTime = SystemClock.elapsedRealtime();

        // Positions assuming center Skystone
        double firstSkystoneY = 2;
        double secondSkyStoneY = 16;
        double secondSkyStoneX = 52;

        Vision.Location skystoneLocation = Vision.Location.UNKNOWN;

        try {
            skystoneLocation = vision.runDetection(true, false);
        }catch (Exception e){}

        telemetry.addLine("Detection Result: " + skystoneLocation.toString());
        telemetry.update();
        sleep(250);

        position2D.startOdometry();

        if (skystoneLocation == Vision.Location.LEFT){
            firstSkystoneY = -5;
            secondSkyStoneY = 6;
            secondSkyStoneX = 52;
        } else if (skystoneLocation == Vision.Location.RIGHT){
            firstSkystoneY = 10;
            secondSkyStoneY = 21;
            secondSkyStoneX = 52;
        }
        double[][] toFirstStone = {
                {0,0,10,0},
                {10,firstSkystoneY,10,0},
                {50,firstSkystoneY,10,0}};
        HashMap<Point,Robot.Actions> toFirstStoneActions = new HashMap<Point,Robot.Actions>(){{
            put(new Point(0,0), Robot.Actions.START_INTAKE);
        }};

        double[][] toFoundation = {
                {50,firstSkystoneY,10,0},
                {32,firstSkystoneY,10,0},
                {26,-10,10,0},
                {21,-30,10,0},
                {23,-87,10,0},
                {41,-99,10,0}};
        HashMap<Point,Robot.Actions> toFoundationActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(24,-45), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(24,-30), Robot.Actions.STOP_INTAKE);
        }};

        double[][] toSecondStone = {
                {41,-99,-10,0},
                {8,-90,-10,0},
                {19,-79,0,10},
                {19,-30,0,10},
                {17,secondSkyStoneY - 5,0,-10},
                {secondSkyStoneX,secondSkyStoneY,30,0}};
        HashMap<Point,Robot.Actions> toSecondStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(28,-99), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(15,-99), Robot.Actions.RELEASE_FOUNDATION);
            put(new Point(28,-10), Robot.Actions.START_INTAKE);
        }};

        double[][] toDepositSecondStone = {
                {secondSkyStoneX,secondSkyStoneY,10,0},
                {30,2,10,-5},
                {28,-33,10,0},
                {28,-50,-10,0},
                {28,-61,10,0},
                {32,-80,10,0}};
        HashMap<Point,Robot.Actions> toDepositSecondStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(28,-30), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(28,-30), Robot.Actions.STOP_INTAKE);
            put(new Point(35,-20), Robot.Actions.START_INTAKE);
        }};

        double[][] toPark = {
                {32,-80,10,10},
                {32,-73,0,10},
                {26,-45,0,10}};
        HashMap<Point,Robot.Actions> toParkActions = new HashMap<Point,Robot.Actions>(){{
            put(new Point(28,-63), Robot.Actions.RETRACT_OUTTAKE);
        }};

        double[][] toThirdStone = {
                {22,-63,5,-10},
                {30 ,-60, 0,10},
                {30,30,0,10},
                {35,-6, 0,10},
                {45,5, 10,0},
                {55,30, 30,0}};
        HashMap<Point,Robot.Actions> toThirdStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(22,-63), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(28,-30), Robot.Actions.START_INTAKE);
        }};

        double[][] toParkAfterThirdStone = {
                {55,30,30,0},
                {30,10,0,10},
                {27,-25,0,10}};
        HashMap<Point,Robot.Actions> toParkAfterThirdStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(42,0), Robot.Actions.STOP_INTAKE);
        }};

        intake(true);

        robot.splineMove(toFirstStone,0.5,1, 0.75,3,0,0,30,
                toFirstStoneActions);

        robot.splineMove(toFoundation,1,1, 1, 10, Math.toRadians(180),Math.toRadians(180),15,
                toFoundationActions, true, 5000);

        robot.foundationMovers(true);
        sleep(350);

        robot.splineMove(toSecondStone,1,1, 1, 20,0,Math.toRadians(60),38,
                toSecondStoneActions,true, 8000);

        robot.splineMove(toDepositSecondStone,0.9,1, 0.4, 10, Math.toRadians(180),Math.toRadians(90),10,
                toDepositSecondStoneActions);

        robot.foundationMovers(false);
        robot.getClamp().setPosition(robot.CLAMP_SERVO_RELEASED);
        robot.brakeRobot();
        sleep(500);

        if (SystemClock.elapsedRealtime() - startTime < 22000) {
            robot.splineMove(toThirdStone, 0.7,1, 0.65, 20,0,Math.toRadians(90),20,
                    toThirdStoneActions, true, 6000);

//            robot.splineMove(toDepositThirdStone, 1, 1, 0.5, 20, Math.toRadians(180), Math.toRadians(270), 10,
//                    toDepositThirdStoneActions);
//
//            retractOuttakeWait();
            robot.splineMove(toParkAfterThirdStone, 1, 1, 0.3, 10, Math.toRadians(180), Math.toRadians(90), 10, toParkAfterThirdStoneActions);
        } else {
            robot.foundationMovers(false);
            robot.splineMove(toPark, 0.7, 1, 0.5, 10, 0, Math.toRadians(90), 5, toParkActions);
        }
    }
}