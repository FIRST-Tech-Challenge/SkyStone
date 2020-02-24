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
        Vucam.findTarget(2.0);
        Vucam.Telemetry();
        telemetry.update();
        // MUST STOP vucam or it will mess up next time it is started
        Vucam.Stop();
        switch (Vucam.Skystone) {
            case LEFT:
                Drive.strafeLeftDistance(speed, 20);
                break;
            case CENTER:
                break;
            case RIGHT:
                Drive.strafeRightDistance(speed, 20);
                break;
        }
        Drive.moveForwardDistance(speed, 20);
        Drive.moveBackwardDistance(speed, 20);
        switch (Vucam.Skystone) {
            case LEFT:
                Drive.strafeRightDistance(speed, 20);
                break;
            case CENTER:
                break;
            case RIGHT:
                Drive.strafeLeftDistance(speed, 20);
                break;
        }
        Drive.moveBackwardDistance(speed, 30);
        telemetry.update();
    }
}
