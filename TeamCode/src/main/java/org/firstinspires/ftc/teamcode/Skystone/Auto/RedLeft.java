package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.MathFunctions;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.PathPoints;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.SplineGenerator;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

@Autonomous(name="Khue", group ="LinearOpmode")
public class RedLeft extends AutoBase{
    // transport two skystones and other stones if time permits

    // park in building zone (even if other team is in corner)
    @Override
    public void runOpMode() {
        initLogic();
        Vision tensorflow = new Vision(robot);


        waitForStart();
        position2D.startOdometry();

        int firstSkystoneX = -2;
        int secondSkyStoneX = -26;

        Vision.Location position = tensorflow.runDetection();

        if (position == Vision.Location.LEFT){
            firstSkystoneX = -5;
            secondSkyStoneX = -29;
        } else if (position == Vision.Location.RIGHT){
            firstSkystoneX = 1;
            secondSkyStoneX = -23;
        }

        double[][] toFirstStone = {{0,0,10,0},{55,firstSkystoneX,10,0}};

        double[][] toFoundation = {{55,firstSkystoneX,-30,0},{24,10,0,10},{24,30,0,10},{24,60,0,10},{36,80,10,0}};

        double[][] toSecondStone = {{31,80,-10,0},{24,60,0,-30},{24,30,0,-10},{24,-15,10,0},{40,secondSkyStoneX,30,0}};

        double[][] toDepositSecondStone = {{40,secondSkyStoneX,-30,0},{24,-10,0,20},{15,30,0,-10},{15,70,0,10}};

        double[][] toPark = {{10,70,0,-10},{10,30,0,-10}};

        intake(true);
        robot.splineMove(toFirstStone,1,1,0,0,15);

        // implement splineMoveWithExtend
        robot.splineMove(toFoundation,1,1,Math.toRadians(180),Math.toRadians(180),30);

        // so you don't need this
        depositStone();
        retractOuttake();

        // and so this works
        robot.foundationMover(true);
        robot.foundationMover(false);

        // implement splineMoveWithRetractAndRelease
        robot.splineMove(toSecondStone,1,1,0,0,10);

        // implement splineMoveWithExtend
        robot.splineMove(toDepositSecondStone,1,1,Math.toRadians(180),Math.toRadians(270),10);
        intake(false);

        // and you don't need this
        depositStone();
        retractOuttake();

        robot.splineMove(toPark,1,1,0,Math.toRadians(270),5);
        return;
    }
}