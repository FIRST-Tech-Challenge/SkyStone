package org.firstinspires.ftc.teamcode.Skystone.Test;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

@Disabled
@Autonomous(name="PositionTest ", group="Linear Opmode")
public class PositionTest  extends LinearOpMode {

    @Override
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap,telemetry,this);
        robot.driveMotorsBreakZeroBehavior();
        robot.resetEncoders();

        waitForStart();
        robot.intializeIMU();
        robot.changeRunModeToUsingEncoder();

        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();

        while (opModeIsActive()){
            sleep(5000);
            break;
        }
    }
}