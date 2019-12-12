package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Robot;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

import java.util.HashMap;

@Autonomous(name="RedFront2", group ="LinearOpmode")
public class RedFront extends AutoBase{
    @Override
    public void runOpMode() {
        long startTime;
        initLogic();

        waitForStart();
        startTime = SystemClock.elapsedRealtime();

        // Positions assuming center Skystone
        double firstSkystoneY = -2;
        double secondSkyStoneY = -18.5;
        double secondSkyStoneX = 54;
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
            secondSkyStoneY = -23;
            secondSkyStoneX = 58;
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
                {35,10,0,10},
                {25,30,0,10},
                {20,60,0,10},
                {33,83.5,10,0}};
        HashMap<Point,Robot.Actions> toFoundationActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(24,45), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(24,30), Robot.Actions.STOP_INTAKE);
        }};

        double[][] toSecondStone = {
                {33,83.5,-10,0},
                {0, 65.5,-10,0},
                {19,64.5,0,-10},
                {21,30,0,-10},
                {28,secondSkyStoneY + 5,0,10},
                {secondSkyStoneX,secondSkyStoneY,30,0}};
        HashMap<Point,Robot.Actions> toSecondStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(20,90), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(-5,80), Robot.Actions.RELEASE_FOUNDATION);
            put(new Point(28,-10), Robot.Actions.START_INTAKE);
        }};

        double[][] toDepositSecondStone = {
                {secondSkyStoneX,secondSkyStoneY,10,0},
                {42,0,0,20},
                {37,-2,0,20},
                {31,30,10,0},
                {31,50,-10,0},
                {29,65,10,0},
                {21,71,10,0}};
        HashMap<Point,Robot.Actions> toDepositSecondStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(28,37), Robot.Actions.EXTEND_OUTTAKE);
            put(new Point(35,-10), Robot.Actions.STOP_INTAKE);
            put(new Point(35,15), Robot.Actions.START_INTAKE);
        }};

        double[][] toPark = {
                {8,77,0,-10},
                {28,75,0,-10},
                {28,35,0,-10}};
        HashMap<Point,Robot.Actions> toParkActions = new HashMap<Point,Robot.Actions>(){{
            put(new Point(8,77), Robot.Actions.RETRACT_OUTTAKE);
        }};

        double[][] toThirdStone = {
                {26,76,0,10},
                {25,60,0,-10},
                {25, 30, 0,-10},
                {35,6,0,-10},
                {45, -5, 10,0},
                {55, -20, 30,0}};
        HashMap<Point,Robot.Actions> toThirdStoneActions = new HashMap<Point,Robot.Actions>() {{
            put(new Point(34,73), Robot.Actions.RETRACT_OUTTAKE);
            put(new Point(28,30), Robot.Actions.START_INTAKE);
        }};

        double[][] toParkAfterThirdStone = {
                {50, -20, 30,0},
                {35,0,0,-10},
                {28,35,0,-10}};
        HashMap<Point,Robot.Actions> toParkAfterThirdStoneActions = new HashMap<Point,Robot.Actions>(){{
            put(new Point(42,-17), Robot.Actions.STOP_INTAKE);
        }};

        intake(true);
        robot.splineMove(toFirstStone,0.5,1, 0.65,3,0,0,20,
                toFirstStoneActions);

        robot.splineMove(toFoundation,0.8,0.8, 0.6, 10, Math.toRadians(180),Math.toRadians(180),15,
                toFoundationActions, true, 10000);

        // get ready to pull foundation
        telemetry.addLine("X: " + robot.getRobotPos().x);
        telemetry.addLine("X: " + robot.getRobotPos().y);
        telemetry.addLine("Angle: " + robot.getAnglePos());
        telemetry.update();
        robot.foundationMovers(true);
        sleep(5000);

        robot.splineMove(toSecondStone,0.8,0.8, 0.6, 20,0,Math.toRadians(320),anglelock,
                toSecondStoneActions);

        robot.splineMove(toDepositSecondStone,0.8,0.8, 0.65, 10, Math.toRadians(180),Math.toRadians(270),20,
                toDepositSecondStoneActions, true, 8000);

        robot.foundationMovers(false);
        robot.getClamp().setPosition(robot.CLAMP_SERVO_RELEASED);

        sleep(400);

        if (SystemClock.elapsedRealtime() - startTime < 22000){
            robot.splineMove(toThirdStone, 1,1, 0.65, 5,0,Math.toRadians(270),20,
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