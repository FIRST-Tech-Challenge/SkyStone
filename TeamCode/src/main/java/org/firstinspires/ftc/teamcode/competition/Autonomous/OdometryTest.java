package org.firstinspires.ftc.teamcode.competition.Autonomous;

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

        /*
          TODO Autonomous drive uses same input as Teleop drive
           * Use Pure-Pursuit as a pathfinder
           * Create some sort of algorithm for using compass-like navigation to get to the next point
           * Compass-pathing uses same MeacnumDrive method "drive" but inputs compass coordinates
             instead of controller input
           * Create a Virtual-Player class which keeps track of everything controlling the robot
           * Use the autonomous class to feed the Virtual-Player anything it needs to know
           * Have a checkpoint system to know when to activate certain mechanisms or do different tasks
        */
    }
}
