package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.MathFunctions;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.PathPoints;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

@Autonomous(name="RedLeft", group ="LinearOpmode")
public class RedLeft extends AutoBase{
    // transport two skystones and other stones if time permits

    // park in building zone (even if other team is in corner)
    @Override
    public void runOpMode() {
        initLogic();
        Vision tensorflow = new Vision(robot);

        waitForStart();
        position2D.startOdometry();
        // Move forward for initial detection of Skystone
        //robot.moveToPoint(6 ,0,1,1,Math.toRadians(0));

        // Start intaking
        intake(true);

        // Run vision to detect where Skystone is located (LEFT, RIGHT, or CENTER)
        Vision.Location position = tensorflow.runDetection();
        telemetry.addLine("position: " + position);
        telemetry.update();

        double firstSkyStoneY;
        double secondSkyStoneY;

        if (position == Vision.Location.LEFT){
            firstSkyStoneY = -6.0;
            secondSkyStoneY = -18.0;
        } else if (position == Vision.Location.CENTER){
            firstSkyStoneY = 0.0;
            secondSkyStoneY = -15.0;
        } else {
            firstSkyStoneY = 6.0;
            secondSkyStoneY = -12.0;
        }

//        double[][] toFirstStone = {{0,0},{24,firstSkyStoneY},{55, firstSkyStoneY}};
//        robot.moveFollowCurve(toFirstStone, 0, 15,0,5);

        robot.moveToPoint(55.0,0,0.7,0.4,0);

        // Store pathpoints required to navigate to foundation. Move through those points and extend outtake.
        double[][] toFoundation = {{55.0,firstSkyStoneY},{15.0,24.0},{15.0,79.0},{37.0,80.0}};
        robot.moveFollowCurveWithExtend(toFoundation, Math.toRadians(-179), 25, Math.toRadians(180), 35, new Point(16.0,60.0));

        robot.foundationMover(true);
        retractOuttake();

        // Store pathpoints required to navigate back to second set of three stones, to pick up second Skystone. Deposit the first Skystone and move via those points.
        double [][] toSecondStone = {{5.0,78.0},{20.0,30.0},{20.0,0}};
        robot.moveFollowCurveWithFoundationMover(toSecondStone, Math.toRadians(0),10,Math.toRadians(-90),0,new Point(16.0, 50.0));

        // Move to actual location of second Skystone
        robot.moveToPoint(55.0,secondSkyStoneY,0.7,1,Math.toRadians(-25));

        // Store pathpoints required to navigate back to foundation, move through those points and extend outtake.
        double[][] toDepositSecondStone = {{40.0,secondSkyStoneY},{15.0,10.0},{15.0,30.0},{17.0,70.0}};
        robot.moveFollowCurveWithExtend(toDepositSecondStone,Math.toRadians(-179),20,Math.toRadians(-90),0,new Point(12.0,50.0));

        // Turn off intake
        intake(false);
        retractOuttake();

        // Store pathpoints required to navigate to parking. Deposit stone and move via those points.
        double[][] toParking = {{17.0,80.0},{15.0,40.0}};
        robot.moveFollowCurve(toParking, Math.toRadians(0),20, Math.toRadians(90),0);
    }
}
