package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;

@Autonomous(name = "Autonomous", group = "Auto")
public class autonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // declare local variables
        double speed = 0.3;

        // display welcome message
        telemetry.setAutoClear(false);
        telemetry.addLine("Autonomous");
        telemetry.update();

        // create and initialize sub-assemblies
        DriveControl Drive = new DriveControl();
        Drive.init(this);

        // wait for PLAY button to be pressed on driver station
        telemetry.addLine(">> Press PLAY to start");
        telemetry.update();
        telemetry.setAutoClear(true);
        waitForStart();

        // begin autonomous actions
        telemetry.setAutoClear(false);

        telemetry.addLine("move forward 2 seconds");
        telemetry.update();
        Drive.moveForwardTime(speed, 2.0);

        telemetry.addLine("wait for 5 seconds");
        telemetry.update();
        Drive.TimeDelay(5.0);

        telemetry.addLine("move backward 100 cm");
        telemetry.update();
        Drive.moveBackwardDistance(speed, 100);

        telemetry.addLine("all done :-)");
        telemetry.update();
        Drive.stop();

        telemetry.setAutoClear(true);
    }
}
