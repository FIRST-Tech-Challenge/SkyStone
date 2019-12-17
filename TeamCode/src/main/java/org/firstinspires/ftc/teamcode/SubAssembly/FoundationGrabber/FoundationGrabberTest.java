package org.firstinspires.ftc.teamcode.SubAssembly.FoundationGrabber;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;

/* Sub Assembly Test OpMode
 * This TeleOp OpMode is used to test the functionality of the specific sub assembly
 */
// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@TeleOp(name = "Foundation Grabber Test", group = "Test")
public class FoundationGrabberTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Foundation Grabber Test: ");

        /* initialize sub-assemblies
         */
        FoundationGrabberControl FoundationGrabber = new FoundationGrabberControl();

        GamepadWrapper egamepad1 = new GamepadWrapper(gamepad1);
        GamepadWrapper egamepad2 = new GamepadWrapper(gamepad2);

        FoundationGrabber.init(this);
        telemetry.update();

        //waits for that giant PLAY button to be pressed on RC
        waitForStart();


        //telling the code to run until you press that giant STOP button on RC
        while (opModeIsActive()) {

            egamepad1.updateEdge();
            egamepad2.updateEdge();

            if (egamepad2.left_bumper.released) {
                FoundationGrabber.grab();
            }


            telemetry.update();

            //let the robot have a little rest, sleep is healthy
            sleep(40);
        }

    }
}
