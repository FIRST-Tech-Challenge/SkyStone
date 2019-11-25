package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.MathFunctions;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.PathPoints;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.SplineGenerator;
import org.firstinspires.ftc.teamcode.Skystone.Robot;
//import org.firstinspires.ftc.teamcode.Skystone.Vision;

@Autonomous(name="RedLeft", group ="LinearOpmode")
public class RedLeft extends AutoBase{
    @Override
    public void runOpMode() {
        initLogic();
//        Vision tensorflow = new Vision(robot);


        waitForStart();
        position2D.startOdometry();

        int firstSkystoneX = -2;
        int secondSkyStoneX = -26;

//        Vision.Location position = tensorflow.runDetection();
//
//        if (position == Vision.Location.LEFT){
//            firstSkystoneX = -5;
//            secondSkyStoneX = -29;
//        } else if (position == Vision.Location.RIGHT){
//            firstSkystoneX = 1;
//            secondSkyStoneX = -23;
//        }

        double[][] toFirstStone = {
                {0,0,10,0},
                {55,firstSkystoneX,10,0}};
        Robot.Actions[] toFirstStoneActions = {};
        Point[] toFirstStoneActionPoints = {};

        double[][] toFoundation = {
                {55,firstSkystoneX,-30,0},
                {24,10,0,10},
                {24,30,0,10},
                {24,55,0,10},
                {31,75,10,0}};
        Robot.Actions[] toFoundationActions = {
                Robot.Actions.EXTEND_OUTTAKE,
                Robot.Actions.STOP_INTAKE};
        Point[] toFoundationActionPoints = {
                new Point(24,30),
                new Point(24,30)};

        double[][] toSecondStone = {
                {31,75,-10,0},
                {8,75,0,-30},
                {24,30,0,-10},
                {24,-15,10,0},
                {55,secondSkyStoneX,30,0}};
        Robot.Actions[] toSecondStoneActions = {
                Robot.Actions.RETRACT_OUTTAKE,
                Robot.Actions.RELEASE_FOUNDATION,
                Robot.Actions.START_INTAKE};
        Point[] toSecondStoneActionPoints = {
                new Point(31,80),
                new Point(20,50),
                new Point(24,55)};

        double[][] toDepositSecondStone = {
                {40,secondSkyStoneX,-30,0},
                {24,-10,0,20},{15,30,0,-10},
                {15,60,0,10}};
        Robot.Actions[] toDepositSecondStoneActions = {
                Robot.Actions.EXTEND_OUTTAKE,
                Robot.Actions.STOP_INTAKE};
        Point[] toDepositSecondStoneActionPoints = {
                new Point(15,20),
                new Point(15,20)};

        double[][] toPark = {{10,60,0,-10},{10,30,0,-10}};
        Robot.Actions[] toParkActions = {};
        Point[] toParkActionPoints = {};

        intake(true);
        robot.splineMove(toFirstStone,0.5,1,0,0,15,
                toFirstStoneActions, toFirstStoneActionPoints);

        robot.splineMove(toFoundation,1,1,Math.toRadians(180),Math.toRadians(180),30,
                toFoundationActions, toFoundationActionPoints);

        // get ready to pull foundation
        robot.foundationMover(true);
        sleep(500);

        robot.splineMove(toSecondStone,1,1,0,Math.toRadians(345),20,
                toSecondStoneActions, toSecondStoneActionPoints);

        robot.splineMove(toDepositSecondStone,1,1,Math.toRadians(180),Math.toRadians(270),10,
                toDepositSecondStoneActions, toDepositSecondStoneActionPoints);

        retractOuttake();
        sleep(800);

        robot.splineMove(toPark,1,1,0,Math.toRadians(270),5, toParkActions, toParkActionPoints);
        return;
    }
}