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

        Vision.Location position = tensorflow.runDetection();

        int firstSkystoneX = -2;
        int secondSkyStoneX = -26;
        if (position == Vision.Location.LEFT){
            firstSkystoneX = -5;
            secondSkyStoneX = -29;
        } else if (position == Vision.Location.RIGHT){
            firstSkystoneX = 1;
            secondSkyStoneX = -23;
        }

        double[][] toFirstStone = {{0,0,10,0},{55,firstSkystoneX,10,0}};

        double[][] toFoundation = {{55,firstSkystoneX,-30,0},{30,10,-10,10},{20,30,0,10},{20,60,0,10},{41,80,10,0}};

        double[][] toSecondStone = {{36,80,-10,0},{17,60,0,-30},{17,30,0,-10},{17,-15,10,0},{40,secondSkyStoneX,30,0}};

        double[][] toDepositSecondStone = {{40,secondSkyStoneX,-30,0},{20,-10,0,20},{20,30,0,-10},{20,70,0,10}};

        double[][] toPark = {{10,70,0,-10},{10,50,0,-10}};

        intake(true);
        robot.splineMove(toFirstStone,1,1,0,0,15,false);

        // implement splineMoveWithExtend
        robot.splineMove(toFoundation,1,1,Math.toRadians(180),Math.toRadians(180),30,true);

        // so you don't need this
        depositStone();
        retractOuttake();

        // and so this works
        robot.foundationMover(true);
        robot.foundationMover(false);

        // implement splineMoveWithRetractAndRelease
        robot.splineMove(toSecondStone,1,1,0,0,20,false);

        // implement splineMoveWithExtend
        robot.splineMove(toDepositSecondStone,1,1,Math.toRadians(180),Math.toRadians(270),30,true);

        robot.splineMove(toPark,1,1,0,Math.toRadians(270),5,false);
        intake(false);
        return;
    }
}