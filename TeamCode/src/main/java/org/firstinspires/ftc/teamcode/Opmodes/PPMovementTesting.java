package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSystems.ActionHandlerClaws;
import org.firstinspires.ftc.teamcode.HardwareSystems.AutoClaws;
import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerKIMU2W;
import org.firstinspires.ftc.teamcode.Movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.RobotPoint;
import org.firstinspires.ftc.teamcode.Movement.Movement;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Timer;

import java.util.ArrayList;

@Autonomous(name="PPTesting 2/23 Afternoon", group="Auto")
public class PPMovementTesting extends LinearOpMode {

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
        telemetry.addData("PP Testing Class:", "running");
        telemetry.update();

//TEST #1: SHORT PATH TEST: RUNS THE SAME PATH WITH 3 DIFFERENT RADII
        /*
        ArrayList <RobotPoint> shortPath = new ArrayList<>();
        shortPath.add(new RobotPoint(0, 0, 0, .8));
        shortPath.add(new RobotPoint(10, 20, 0, .8));
        shortPath.add(new RobotPoint(15, 50, 0, .8));
        movement.followPath(shortPath, 30);
        telemetry.addData("Status", "Done First Run");
        telemetry.update();
        timer.waitMillis(3000);
         */

//TEST #2: LONG PATH TEST: RUNS A LONG SMOOTH ARC GOING FORWARDS THEN BACKWARDS

        ArrayList <RobotPoint> longPath = new ArrayList<>();
        longPath.add(new RobotPoint(-5, 30, 0, .8));
        longPath.add(new RobotPoint(-20, 60, 0, .8));
        longPath.add(new RobotPoint(-35, 90, 0, .8));
        longPath.add(new RobotPoint(-50, 140, 0, .8));
        longPath.add(new RobotPoint(-40, 160, 0, .8));
        longPath.add(new RobotPoint(-30, 180, 0, .8));
        longPath.add(new RobotPoint(-10, 200, 0, .8));
        movement.followPath(longPath, 20);
        timer.waitMillis(1000);


        ArrayList <RobotPoint> longBackwardsPath = new ArrayList<>();
        longBackwardsPath.add(new RobotPoint(-10, 200, 0, .8));
        longBackwardsPath.add(new RobotPoint(-30, 180, 0, .8));
        longBackwardsPath.add(new RobotPoint(-40, 160, 0, .8));
        longBackwardsPath.add(new RobotPoint(-50, 140, 0, .8));
        longBackwardsPath.add(new RobotPoint(-35, 90, 0, .8));
        longBackwardsPath.add(new RobotPoint(-20, 60, 0, .8));
        longBackwardsPath.add(new RobotPoint(-5, 30, 0, .8));
        movement.followPath(longBackwardsPath, 20);
        timer.waitMillis(50000);

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

        autoClaws.initialize();
        drivetrain.initialize();
        odometer.initialize();

        telemetry.addData("status","initialized");
        telemetry.update();

    }
}
