package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Robot;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

import java.util.HashMap;

@Autonomous(name="RedFront", group ="LinearOpmode")
public class RedFront extends AutoBase{
    @Override
    public void runOpMode() {
        long startTime;
        initLogic();

        waitForStart();
        startTime = SystemClock.elapsedRealtime();

        // Positions assuming center Skystone
        double firstSkystoneY = -2;
        double secondSkyStoneY = -15;
        double secondSkyStoneX = 69;
        double anglelock = 33;
        Vision.Location skystoneLocation = Vision.Location.UNKNOWN;
        try {
            skystoneLocation = vision.runDetection(true, true);
        }catch (Exception e){

        }

        telemetry.addLine("Detection Result: " + skystoneLocation.toString());
        telemetry.update();

        sleep(250);

        position2D.startOdometry();

        // Change Skystone positions if detected left or right
        if (skystoneLocation == Vision.Location.LEFT){
            firstSkystoneY = -7;
            secondSkyStoneY = -23.5;
            secondSkyStoneX = 69;
            anglelock = 36;
        } else if (skystoneLocation == Vision.Location.RIGHT){
            firstSkystoneY = 5;
            secondSkyStoneY = -8;
        }

        double[][] toFirstStone = {
                {0,0,10,0},
                {10,firstSkystoneY,10,0},
                {50,firstSkystoneY,10,0}};
        HashMap<Point,Robot.Actions> toFirstStoneActions = new HashMap<Point,Robot.Actions>(){{
            put(new Point(0,0), Robot.Actions.START_INTAKE);
        }};

        double[][] toFoundation = {
                {55,firstSkystoneY,-30,0},
                {28,10,0,10},
                {27,30,0,10},
                {26,80,0,10},
                {35,85,10,0}};
        HashMap<Point,Robot.Actions> toFoundationActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(24,45), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(24,30), Robot.Actions.STOP_INTAKE);
        }};

        double[][] toSecondStone = {
                {31,83,-10,0},
                {8, 72,0,-10},
                {32,71,0,10},
                {35,30,0,10},
                {39,secondSkyStoneY + 15.5,0,10},
                {secondSkyStoneX,secondSkyStoneY,30,0}};
        HashMap<Point,Robot.Actions> toSecondStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(32,80), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(24,58.5), Robot.Actions.RELEASE_FOUNDATION);
            put(new Point(28,-10), Robot.Actions.START_INTAKE);
        }};

        double[][] toDepositSecondStone = {
                {55,secondSkyStoneY,-30,0},
                {46,-10,10,20},
                {42,30,5,10},
                {42,50,5,10},
                {41,65,5,10},
                {35,76.5,0,10}};
        HashMap<Point,Robot.Actions> toDepositSecondStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(40,37), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(30,-10), Robot.Actions.STOP_INTAKE);
            put(new Point(30,5), Robot.Actions.START_INTAKE);
        }};

        double[][] toPark = {
                {35,77,0,-10},
                {38,75,0,-10},
                {40,35,0,-10}};
        HashMap<Point,Robot.Actions> toParkActions = new HashMap<Point,Robot.Actions>(){{
            put(new Point(34,73), Robot.Actions.RETRACT_OUTTAKE);
        }};

        double[][] toThirdStone = {
                {36,76,0,10},
                {42,60,0,-10},
                {39, 30, 0,-10},
                {45,6,0,-10},
                {70, -5, 10,0},
                {82, -20, 30,0}};
        HashMap<Point,Robot.Actions> toThirdStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(34,73), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(28,30), Robot.Actions.START_INTAKE);
        }};

        double[][] toParkAfterThirdStone = {
                {82, -20, 30,0},
                {52,0,0,-10},
                {45,35,0,-10}};
        HashMap<Point,Robot.Actions> toParkAfterThirdStoneActions = new HashMap<Point,Robot.Actions>(){{
            put(new Point(42,-17), Robot.Actions.STOP_INTAKE);
        }};

        intake(true);
        robot.splineMove(toFirstStone,0.5,1, 0.65,3,0,0,20,
                toFirstStoneActions);

        robot.splineMove(toFoundation,1,1, 1, 10, Math.toRadians(180),Math.toRadians(180),30,
                toFoundationActions);

        // get ready to pull foundation
        robot.foundationMovers(true);
        sleep(250);

        robot.splineMove(toSecondStone,1,1, 0.85, 20,0,Math.toRadians(325),anglelock,
                toSecondStoneActions);

        robot.splineMove(toDepositSecondStone,1,1, 0.65, 10, Math.toRadians(180),Math.toRadians(270),20,
                toDepositSecondStoneActions);

        robot.foundationMovers(false);
        robot.getClamp().setPosition(robot.CLAMP_SERVO_RELEASED);

        sleep(250);

        if (SystemClock.elapsedRealtime() - startTime < 22000){
            robot.splineMove(toThirdStone, 0.7,1, 0.65, 5,0,Math.toRadians(270),20,
                    toThirdStoneActions,true,6000);

//            robot.splineMove(toDepositThirdStone, 1, 1, 0.5, 20, Math.toRadians(180), Math.toRadians(270), 10,
//                    toDepositThirdStoneActions);
//
//            retractOuttakeWait();
            robot.splineMove(toParkAfterThirdStone, 1, 1, 0.3, 10, Math.toRadians(180), Math.toRadians(270), 20, toParkAfterThirdStoneActions);
        } else {
            robot.splineMove(toPark, 0.6, 1, 0.4, 5, 0, Math.toRadians(270), 5, toParkActions);
        }
    }
}