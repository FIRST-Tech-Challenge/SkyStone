package org.firstinspires.ftc.teamcode.SubAssembly.Claimer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;

/* Sub Assembly Test OpMode
 * This TeleOp OpMode is used to test the functionality of the specific sub assembly
 */
// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@TeleOp(name = "Grabber Test", group = "Test")
public class GrabberTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Grabber Test: ");

        /* initialize sub-assemblies
         */
        GrabberControl Grabber = new GrabberControl();

        GamepadWrapper egamepad1 = new GamepadWrapper(gamepad1);
        GamepadWrapper egamepad2 = new GamepadWrapper(gamepad2);

        Grabber.init(this);
        telemetry.update();

        //waits for that giant PLAY button to be pressed on RC
        waitForStart();


        //telling the code to run until you press that giant STOP button on RC
        while (opModeIsActive()) {

            egamepad1.updateEdge();
            egamepad2.updateEdge();


            if (egamepad1.a.released) {
                Grabber.open();
            } else if (egamepad1.b.released) {
                Grabber.close();
            }

            if (egamepad1.x.released) {

                for(int p=0;p<180;p++){
                Grabber.wrist(p);
                wait(25);
            }

            }


            //SubAssembly.test();
            telemetry.update();

            //let the robot have a little rest, sleep is healthy
            sleep(40);
        }

    }
}
