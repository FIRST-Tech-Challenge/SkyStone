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

@Autonomous(name="RedLeft", group ="LinearOpmode")
public class RedLeft extends AutoBase{
    @Override
    public void runOpMode() {
        long startTime;
        initLogic();

        Vision tensorflow = new Vision(robot);

        waitForStart();
        startTime = SystemClock.elapsedRealtime();
        position2D.startOdometry();

        // this will be the center positions
        int firstSkystoneX = -2;
        int secondSkyStoneX = -20;
        int secondSkystoneY = 65;

        Vision.Location skystoneLocation = tensorflow.runDetection();

        if (skystoneLocation == Vision.Location.LEFT){
            firstSkystoneX = -7;
            secondSkyStoneX = -27;
            secondSkystoneY = 48;
        } else if (skystoneLocation == Vision.Location.RIGHT){
            firstSkystoneX = 5;
            secondSkyStoneX = -13;
        }

        double[][] toFirstStone = {
                {0,0,10,0},
                {10,firstSkystoneX,10,0},
                {50,firstSkystoneX,10,0}};
        HashMap<Point,Robot.Actions> toFirstStoneActions = new HashMap<Point,Robot.Actions>();

        double[][] toFoundation = {
                {55,firstSkystoneX,-30,0},
                {26,10,0,10},
                {26,30,0,10},
                {24,80,0,10},
                {33,80,10,0}};
        HashMap<Point,Robot.Actions> toFoundationActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(24,40), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(24,30), Robot.Actions.STOP_INTAKE);
        }};

        double[][] toSecondStone = {
                {31,75,-10,0},
                {10, 70,0,-10},
                {26,57,0,10},
                {26,30,0,10},
                {26,secondSkyStoneX + 5,10,0},
                {secondSkystoneY,secondSkyStoneX,30,0}};
        HashMap<Point,Robot.Actions> toSecondStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(32,80), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(20,55), Robot.Actions.RELEASE_FOUNDATION);
            put(new Point(28,55), Robot.Actions.START_INTAKE);
        }};

        double[][] toDepositSecondStone = {
                {55,secondSkyStoneX,-30,0},
                {30,-10,0,20},
                {30,30,0,-10},
                {17,75,0,10}};
        HashMap<Point,Robot.Actions> toDepositSecondStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(15,32), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(30,30), Robot.Actions.STOP_INTAKE);
        }};

        double[][] toPark = {
                {15,60,0,-10},
                {28,50,0,-10},
                {29,30,0,-10}};
        HashMap<Point,Robot.Actions> toParkActions = new HashMap<Point,Robot.Actions>();

        double[][] toThirdStone = {
                {16,60,0,-10},
                {28, 30, 0,-10},
                {24, -10, 10,0},
                {55, -20, 30,0}};
        HashMap<Point,Robot.Actions> toThirdStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(28,30), Robot.Actions.START_INTAKE);
        }};

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
        robot.splineMove(toFirstStone,0.5,1, 0.5,3,0,0,30,
                toFirstStoneActions);

        robot.splineMove(toFoundation,1,1, 0.5, 10, Math.toRadians(180),Math.toRadians(180),30,
                toFoundationActions);

        // get ready to pull foundation
        robot.foundationMover(true);
        sleep(500);

        robot.splineMove(toSecondStone,1,1, 0.5, 20,0,Math.toRadians(345),30,
                toSecondStoneActions);

        robot.splineMove(toDepositSecondStone,0.9,1, 0.5, 10, Math.toRadians(180),Math.toRadians(270),10,
                toDepositSecondStoneActions);

        retractOuttakeWait();
        sleep(800);

        if (SystemClock.elapsedRealtime() - startTime < 20000){
            robot.splineMove(toThirdStone, 1,1, 0.5, 20,0,0,20,
                    toThirdStoneActions);

            robot.splineMove(toDepositThirdStone, 1, 1, 0.5, 20, Math.toRadians(180), Math.toRadians(270), 10,
                    toDepositThirdStoneActions);

            retractOuttakeWait();
        }

        robot.splineMove(toPark,1,1, 0.3, 10,0,Math.toRadians(270),5, toParkActions);
        return;
    }
}