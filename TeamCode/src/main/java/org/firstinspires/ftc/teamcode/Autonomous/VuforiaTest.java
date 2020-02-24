package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Vucam.VucamControl;

// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@Autonomous(name = "Vuforia Test", group = "Test")
public class VuforiaTest extends LinearOpMode {

    DriveControl Drive = new DriveControl();
    VucamControl Vucam = new VucamControl();

    @Override
    public void runOpMode() throws InterruptedException {
        // declare local variables
        double speed = 0.3;

        // display welcome message
        telemetry.setAutoClear(false);
        telemetry.addLine("Vuforia Test");

        // create and initialize sub-assemblies
        Drive.init(this);
        Vucam.init(this);

        // wait for PLAY button to be pressed on driver station
        telemetry.addLine(">> Press PLAY to start");
        telemetry.update();
        telemetry.setAutoClear(true);
        waitForStart();

        // don't start until targets are randomized
        Vucam.Start();

        Drive.moveForwardDistance(speed, 30);
        Vucam.findTarget(1.0);
        // MUST STOP vucam or it will mess up next time it is started
        Vucam.Stop();
        switch (Vucam.Skystone) {
            case INIT:
                telemetry.addLine("Initial");
                break;
            case LEFT:
                telemetry.addLine("Left");
                break;
            case CENTER:
                telemetry.addLine("Center");
                break;
            case RIGHT:
                telemetry.addLine("Right");
                break;
        }

        Drive.moveBackwardDistance(speed, 30);
        telemetry.update();
    }
}
