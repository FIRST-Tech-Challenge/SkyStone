package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.MathFunctions;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.PathPoints;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.SplineGenerator;

@Autonomous(name="Khue", group ="LinearOpmode")
public class RedLeft extends AutoBase{
    // transport two skystones and other stones if time permits

    // park in building zone (even if other team is in corner)
    @Override
    public void runOpMode() {
        initLogic();
        double[][] points = {{0,0,2.0,0},{20.0,0.0,5,0},{50.0, -35.0, 10.0,5.0}};
        double[][] pointsBackwards = {{0,0,-10,0},{-70,0,-5,0},{-90.0,30,-20,10}};


        waitForStart();
        position2D.startOdometry();

        robot.splineMove(points,1,1,Math.toRadians(180),Math.toRadians(270),15,false);

        sleep(10000);

        return;
    }
}