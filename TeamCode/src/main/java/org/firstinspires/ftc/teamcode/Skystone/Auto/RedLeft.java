package org.firstinspires.ftc.teamcode.Skystone.Auto;

import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class RedLeft extends AutoBase{
    // move foundation and get more skystones and park

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap,telemetry,this);
        robot.driveMotorsBreakZeroBehavior();
        robot.resetEncoders();
        robot.intializeIMU();
        robot.changeRunModeToUsingEncoder();
        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();
        waitForStart();
        int vuforiaPosition = robot.detectTensorflow();
        robot.goToSkystone(vuforiaPosition);
        robot.moveToPoint(7, -47, 1,  1, Math.toRadians(90));
        extendOuttake(robot);
        retractOuttake(robot);
        robot.moveToPoint(0,0,1,1,Math.toRadians(0));
        robot.goToSkystone(vuforiaPosition);
    }

}
