package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.competition.Hardware;
import org.firstinspires.ftc.teamcode.competition.MecanumDrive;

/**
 * Testing out the odometry
 */
@TeleOp(name = "OdometryTest", group = "Auto")
public class OdometryTest extends LinearOpMode {

    private Hardware robot;
    private MecanumDrive driveTrain;

    @Override
    public void runOpMode() {
        // Sets up classes
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);

        // Initializes robot
        robot.init(hardwareMap);
    }
}
