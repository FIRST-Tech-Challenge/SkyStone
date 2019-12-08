package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.MathFunctions;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.PathPoints;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.SplineGenerator;
import org.firstinspires.ftc.teamcode.Skystone.Robot;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

import java.util.HashMap;

@Autonomous(name="BlueFront", group ="LinearOpmode")
public class BlueFront extends AutoBase{
    @Override
    public void runOpMode() {
        long startTime;
        initLogic();

        waitForStart();
        startTime = SystemClock.elapsedRealtime();

        // Positions assuming center Skystone
        double firstSkystoneY = 2;
        double secondSkyStoneY = 14;
        double secondSkyStoneX = 69;

        Vision.Location skystoneLocation = Vision.Location.UNKNOWN;

        try {
            skystoneLocation = vision.runDetection(true);
        }catch (Exception e){

        }
        telemetry.addLine("Detection Result: " + skystoneLocation.toString());
        telemetry.update();
        sleep(250);

        position2D.startOdometry();

        if (skystoneLocation == Vision.Location.LEFT){
            firstSkystoneY = -5;
            secondSkyStoneY = 13.25;
        } else if (skystoneLocation == Vision.Location.RIGHT){
            firstSkystoneY = 7;
            secondSkyStoneY = 21;
            secondSkyStoneX = 69;
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
                {30,-10,0,-10},
                {32,-30,0,-10},
                {33,-80,0,-10},
                {38,-85,10,0}};
        HashMap<Point,Robot.Actions> toFoundationActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(40,-60), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(24,-30), Robot.Actions.STOP_INTAKE);
        }};

        double[][] toSecondStone = {
                {31,-83,-10,0},
                {9,-70,-10,0},
                {32,-69,10,0},
                {31,-30,10,0},
                {39,secondSkyStoneY - 15.5,10,0},
                {secondSkyStoneX,secondSkyStoneY,30,0}};
        HashMap<Point,Robot.Actions> toSecondStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(32,-78), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(24,-59), Robot.Actions.RELEASE_FOUNDATION);
            put(new Point(28,-10), Robot.Actions.START_INTAKE);
        }};

        double[][] toDepositSecondStone = {
                {55,secondSkyStoneY,-30,0},
                {47,10,10,0},
                {44,-30,10,-20},
                {43,-50,5,10},
                {41,-65,5,10},
                {36,-77,5,-10}};
        HashMap<Point,Robot.Actions> toDepositSecondStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(34,-43), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(30,0), Robot.Actions.STOP_INTAKE);
            put(new Point(30,-10), Robot.Actions.START_INTAKE);
        }};

        double[][] toPark = {
                {36,-77,10,0},
                {45,-75,10,0},
                {45,-35,0,10}};
        HashMap<Point,Robot.Actions> toParkActions = new HashMap<Point,Robot.Actions>(){{
            put(new Point(40,-74), Robot.Actions.RETRACT_OUTTAKE);
        }};

        double[][] toThirdStone = {
                {36,-76,5,10},
                {42 ,-60, 15,0},
                {39,30,-10,0},
                {45,6, 10,0},
                {70,5, 10,0},
                {82,20, 30,0}};
        HashMap<Point,Robot.Actions> toThirdStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(40,-74), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(28,-30), Robot.Actions.START_INTAKE);
        }};

        double[][] toParkAfterThirdStone = {
                {32,20,10,0},
                {42,-30,10,0}};
        HashMap<Point,Robot.Actions> toParkAfterThirdStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(37,17), Robot.Actions.STOP_INTAKE);
        }};

        intake(true);

        robot.splineMove(toFirstStone,0.5,1, 0.5,3,0,0,30,
                toFirstStoneActions);

        robot.splineMove(toFoundation,1,1, 0.5, 10, Math.toRadians(180),Math.toRadians(180),30,
                toFoundationActions);

        // get ready to pull foundation
        robot.foundationMover(true);
        sleep(250);

        robot.splineMove(toSecondStone,1,1, 0.85, 20,0,Math.toRadians(35),33,
                toSecondStoneActions);

        robot.splineMove(toDepositSecondStone,0.9,1, 0.65, 10, Math.toRadians(180),Math.toRadians(90),10,
                toDepositSecondStoneActions);

        robot.getClamp().setPosition(robot.CLAW_SERVO_RELEASED);

        sleep(250);

        if (SystemClock.elapsedRealtime() - startTime < 22000){
            robot.splineMove(toThirdStone, 0.7,1, 0.65, 20,0,Math.toRadians(90),20,
                    toThirdStoneActions, true, 6000);

//            robot.splineMove(toDepositThirdStone, 1, 1, 0.5, 20, Math.toRadians(180), Math.toRadians(270), 10,
//                    toDepositThirdStoneActions);
//
//            retractOuttakeWait();
            robot.splineMove(toParkAfterThirdStone, 1, 1, 0.3, 10, Math.toRadians(180), Math.toRadians(90), 10, toParkAfterThirdStoneActions);


        }else {
            robot.splineMove(toPark, 0.7, 1, 0.3, 10, 0, Math.toRadians(90), 5, toParkActions);
        }
    }
}