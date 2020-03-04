package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSystems.ActionHandler;
import org.firstinspires.ftc.teamcode.HardwareSystems.ActionHandlerClaws;
import org.firstinspires.ftc.teamcode.HardwareSystems.AutoClaws;
import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerIMU2W;
import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerKIMU2W;
import org.firstinspires.ftc.teamcode.Movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.RobotPoint;
import org.firstinspires.ftc.teamcode.Movement.Movement;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Timer;

import java.util.ArrayList;

@Autonomous(name="Stuck In Stop Test", group="Testing")
@Disabled
public class crashTest extends LinearOpMode {

    // Declare OpMode Members
    private RobotHardware hardware = new RobotHardware();
    private Timer timer;
    private OdometerKIMU2W odometer;
    private MecanumDrive drivetrain;
    private Movement movement;
    private AutoClaws autoClaws;
    private ActionHandlerClaws handler;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        timer.start();
        odometer.startTracking(0, 0, 0);
        telemetry.addData("status","running");
        telemetry.update();

        timer.waitMillis(5000);

    }

    private void initialize(){
        hardware.hardwareMap(hardwareMap);

        drivetrain = new MecanumDrive(this, hardware);
        odometer = new OdometerKIMU2W(this, hardware);
        timer = new Timer(this, odometer);
        autoClaws = new AutoClaws(hardware, "BLUE", timer);
        handler = new ActionHandlerClaws(autoClaws);
        movement = new Movement(this, drivetrain, odometer, timer);
        movement.setActionHandler(handler);
        movement.useActionHandler = true;

        hardware.foundationClampLeft.setPosition(0.745);
        hardware.foundationClampRight.setPosition(0.26);

        autoClaws.initialize();
        drivetrain.initialize();
        odometer.initialize();

        telemetry.addData("status","initialized");
        telemetry.update();
    }
}
