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

@Autonomous(name="RedFront", group ="LinearOpmode")
public class RedFront extends AutoBase{
    @Override
    public void runOpMode() {
        long startTime;
        initLogic();

        waitForStart();
        startTime = SystemClock.elapsedRealtime();

        // Positions assuming center Skystone
        int firstSkystoneY = -2;
        int secondSkyStoneY = -20;
        int secondSkyStoneX = 67;
        Vision.Location skystoneLocation = Vision.Location.UNKNOWN;
        try {
            skystoneLocation = vision.runDetection3(true);
        }catch (Exception e){

        }

        telemetry.addLine("Detection Result: " + skystoneLocation.toString());
        telemetry.update();

        sleep(250);

        position2D.startOdometry();

        // Change Skystone positions if detected left or right
        if (skystoneLocation == Vision.Location.LEFT){
            firstSkystoneY = -7;
            secondSkyStoneY = -25;
            secondSkyStoneX = 53;
        } else if (skystoneLocation == Vision.Location.RIGHT){
            firstSkystoneY = 5;
            secondSkyStoneY = -10;
        }

        double[][] toFirstStone = {
                {0,0,10,0},
                {10,firstSkystoneY,10,0},
                {50,firstSkystoneY,10,0}};
        HashMap<Point,Robot.Actions> toFirstStoneActions = new HashMap<Point,Robot.Actions>();

        double[][] toFoundation = {
                {55,firstSkystoneY,-30,0},
                {28,10,0,10},
                {28,30,0,10},
                {26,80,0,10},
                {34,90,10,0}};
        HashMap<Point,Robot.Actions> toFoundationActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(24,45), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(24,30), Robot.Actions.STOP_INTAKE);
        }};

        double[][] toSecondStone = {
                {31,90,-10,0},
                {8, 75,0,-10},
                {30,65,0,10},
                {30,30,0,10},
                {32,secondSkyStoneY + 5,10,0},
                {secondSkyStoneX,secondSkyStoneY,30,0}};
        HashMap<Point,Robot.Actions> toSecondStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(32,80), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(20,60), Robot.Actions.RELEASE_FOUNDATION);
            put(new Point(28,55), Robot.Actions.START_INTAKE);
        }};

        double[][] toDepositSecondStone = {
                {55,secondSkyStoneY,-30,0},
                {47,-10,10,20},
                {39,30,5,10},
                {38,76,0,10}};
        HashMap<Point,Robot.Actions> toDepositSecondStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(40,37), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(30,-10), Robot.Actions.STOP_INTAKE);
            put(new Point(30,0), Robot.Actions.START_INTAKE);
        }};

        double[][] toPark = {
                {15,60,0,-10},
                {28,50,0,-10},
                {37,30,0,-10}};
        HashMap<Point,Robot.Actions> toParkActions = new HashMap<Point,Robot.Actions>(){{
            put(new Point(40,73), Robot.Actions.RETRACT_OUTTAKE);
        }};

        double[][] toThirdStone = {
                {16,60,0,-10},
                {32, 30, 0,-10},
                {47,6,0,-10},
                {54, -10, 10,0},
                {60, -24, 30,0}};
        HashMap<Point,Robot.Actions> toThirdStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(40,73), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(28,30), Robot.Actions.START_INTAKE);
        }};

        double[][] toParkAfterThirdStone = {
                {60, -20, 30,0},
                {45,0,0,-10},
                {37,40,0,-10}};
        HashMap<Point,Robot.Actions> toParkAfterThirdStoneActions = new HashMap<Point,Robot.Actions>();

        double[][] toDepositThirdStone = {
                {55,-20,-30,0},
                {24,-10,0,20},
                {15,30,0,-10},
                {15,60,0,10}};
        HashMap<Point,Robot.Actions> toDepositThirdStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(15,30), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(15,20), Robot.Actions.STOP_INTAKE);
        }};

        intake(true);
        robot.splineMove(toFirstStone,0.5,1, 0.4,3,0,0,30,
                toFirstStoneActions);

        robot.splineMove(toFoundation,1,1, 0.5, 10, Math.toRadians(180),Math.toRadians(180),30,
                toFoundationActions);

        // get ready to pull foundation
        robot.foundationMover(true);
        sleep(250);

        robot.splineMove(toSecondStone,1,1, 0.85, 20,0,Math.toRadians(345),30,
                toSecondStoneActions);

        robot.splineMove(toDepositSecondStone,0.9,1, 0.3, 10, Math.toRadians(180),Math.toRadians(270),10,
                toDepositSecondStoneActions);
        robot.getClamp().setPosition(robot.CLAW_SERVO_RELEASED);

        sleep(250);

        if (SystemClock.elapsedRealtime() - startTime < 24000){
            robot.splineMove(toThirdStone, 0.7,1, 0.65, 5,0,Math.toRadians(270),20,
                    toThirdStoneActions);

//            robot.splineMove(toDepositThirdStone, 1, 1, 0.5, 20, Math.toRadians(180), Math.toRadians(270), 10,
//                    toDepositThirdStoneActions);
//
//            retractOuttakeWait();
            robot.splineMove(toParkAfterThirdStone, 1, 1, 0.4, 10, Math.toRadians(180), Math.toRadians(270), 20, toParkAfterThirdStoneActions);
        } else {
            robot.splineMove(toPark, 0.7, 1, 0.5, 5, 0, Math.toRadians(270), 5, toParkActions);
        }
    }
}