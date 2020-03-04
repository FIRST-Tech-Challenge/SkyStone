package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerIMU2W;
import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerKIMU2W;
import org.firstinspires.ftc.teamcode.Movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.Movement.Movement;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Timer;

@Autonomous(name="Refresh Test", group="Testing")
@Disabled
public class refreshTest extends LinearOpMode {

    // Declare OpMode Members
    private RobotHardware hardware = new RobotHardware();
    private OdometerIMU2W odometer;
    //private OdometerKIMU2W odometer;
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

        int loopCounter = 0;
        while(loopCounter < 1000){
            odometer.update();

            drivetrain.update();
            movement.setGlobalVelocity(0, 0, 0);

            loopCounter ++;

        }
        double loopTime = timer.getTimeMillis()/1000; //Average loop time in milliseconds
        telemetry.addData("Refresh Rate", loopTime);
        telemetry.update();
        timer.waitMillis(5000);

    }

    private void initialize(){
        hardware.hardwareMap(hardwareMap);

        odometer = new OdometerIMU2W(this, hardware);
        //odometer = new OdometerKIMU2W();
        drivetrain = new MecanumDrive(this, hardware);
        timer = new Timer(this, odometer);
        movement = new Movement(this, drivetrain, odometer, timer);

        drivetrain.initialize();
        odometer.initialize();

        telemetry.addData("status","initialized");
        telemetry.update();

    }
}
