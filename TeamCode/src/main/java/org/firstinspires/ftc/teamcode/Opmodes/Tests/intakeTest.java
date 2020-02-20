package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSystems.Extrusion;
import org.firstinspires.ftc.teamcode.HardwareSystems.Intake;
import org.firstinspires.ftc.teamcode.HardwareSystems.Outtake;
import org.firstinspires.ftc.teamcode.Movement.Localization.Odometer;
import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerIMU2W;
import org.firstinspires.ftc.teamcode.Movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.Movement.Movement;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Timer;

@Autonomous(name="Intake Test", group="Testing")
public class intakeTest extends LinearOpMode {

    // Declare OpMode Members
    private OdometerIMU2W odometer;
    private MecanumDrive drivetrain;
    private Movement movement;
    private Timer timer;
    private Intake intake;
    private Outtake outtake;
    private Extrusion extrusion;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        timer.start();
        odometer.startTracking(0, 0, 0);
        telemetry.addData("status","running");
        telemetry.update();

        intake.setPower(0.6);
        timer.waitMillis(3000);
        intake.setPower(0);
        timer.waitMillis(1000);
        outtake.setGripperState("Clamped");
        timer.waitMillis(1000);
        extrusion.setPower(0.5);
        timer.waitMillis(5000);

    }

    private void initialize(){
        RobotHardware.hardwareMap(hardwareMap);

        odometer = new OdometerIMU2W();
        drivetrain = new MecanumDrive();
        timer = new Timer(this, odometer);
        movement = new Movement(this, drivetrain, odometer);
        intake = new Intake();
        outtake = new Outtake();
        extrusion = new Extrusion(this);

        drivetrain.initialize();
        odometer.initialize();
        intake.initialize();
        outtake.initialize();
        extrusion.initialize();

        telemetry.addData("status","initialized");
        telemetry.update();

    }
}
