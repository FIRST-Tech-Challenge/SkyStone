package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerIMU2W;
import org.firstinspires.ftc.teamcode.Movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.RobotPoint;
import org.firstinspires.ftc.teamcode.Movement.Movement;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import java.util.ArrayList;
@Disabled
@Autonomous(name="Movement Test", group="Testing")

public class movementTest extends LinearOpMode {

    // Declare OpMode Members
    private RobotHardware hardware = new RobotHardware();
    private OdometerIMU2W odometer;
    private MecanumDrive drivetrain;
    private Movement movement;
    private Timer timer;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        timer.start();
        odometer.startTracking(0, 0, 0);
        telemetry.addData("status","running");
        telemetry.update();

     //   movement.pointInDirection(90, 10);
        movement.deadReckon(0,-0.5,0,1000);
        timer.waitMillis(2000);
        movement.deadReckon(0,0.5,0.5,1000);
        timer.waitMillis(2000);
        movement.deadReckon(0.5,0.5,0,1000);


        drivetrain.freeze();

    }

    private void initialize(){
        hardware.hardwareMap(hardwareMap);

        odometer = new OdometerIMU2W(this, hardware);
        drivetrain = new MecanumDrive(this, hardware);
        timer = new Timer(this, odometer);
        movement = new Movement(this, drivetrain, odometer, timer);

        drivetrain.initialize();
        odometer.initialize();

        telemetry.addData("status","initialized");
        telemetry.update();

    }
}
