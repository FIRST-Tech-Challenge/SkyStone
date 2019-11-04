package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

@Autonomous(name="BlueLeft", group ="LinearOpmode")
public class BlueLeft extends AutoBase {
    // transport two skystones and other stones if time permits

    // park in building zone (even if other team is in corner)
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap,telemetry,this);
        robot.driveMotorsBreakZeroBehavior();
        robot.resetEncoders();
        waitForStart();
        robot.intializeIMU();
        robot.changeRunModeToUsingEncoder();
        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();

//        robot.goToSkystone();
        robot.moveToPoint(0, -30, 1, 0, Math.toRadians(20));
        telemetry.addLine("return");
        telemetry.update();
        telemetry.addLine("DONEEEEE");
        telemetry.update();
        extendOuttake(robot);
        retractOuttake(robot);
        robot.moveToPoint(0,0,1,1,Math.toRadians(0));
    }
}