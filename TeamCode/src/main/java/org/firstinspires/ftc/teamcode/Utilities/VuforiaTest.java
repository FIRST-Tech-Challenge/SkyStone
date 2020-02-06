package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
import org.firstinspires.ftc.teamcode.Utilities.ConceptVuforiaSkyStoneNavigationWebcam;
import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;

/* Sub Assembly Test OpMode
 * This TeleOp OpMode is used to test the functionality of the specific sub assembly
 */

// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@TeleOp(name = "Vuforia Test", group = "Test")
public class VuforiaTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // create and initialize sub-assemblies
        DriveControl Drive = new DriveControl();
        ConceptVuforiaSkyStoneNavigationWebcam Webcam = new ConceptVuforiaSkyStoneNavigationWebcam();
        Drive.init(this);

        // wait for PLAY button to be pressed on driver station
        telemetry.addLine(">> Press PLAY to start");
        telemetry.update();
        telemetry.setAutoClear(true);
        waitForStart();

        // let the robot have a little rest, sleep is healthy
        sleep(40);
    }
}
