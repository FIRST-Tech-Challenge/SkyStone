package org.firstinspires.ftc.teamcode.SubAssembly.Vucam;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Vucam Test", group = "Test")
public class VucamTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Vucam  Test OpMode");
        telemetry.update();

        /* initialize sub-assemblies
         */
        VucamControl Vucam = new VucamControl();

        Vucam.init(this);

        //waits for that giant PLAY button to be pressed on RC
        telemetry.addLine(">> Press PLAY to start");
        telemetry.update();
        telemetry.setAutoClear(true);
        waitForStart();

        Vucam.setSamplePos();

        //telling the code to run until you press that giant STOP button on RC
        while (opModeIsActive()) {

            Vucam.Telemetry();
            telemetry.update();

            //let the robot have a little rest, sleep is healthy
            sleep(40);
        }

        Vucam.Stop();
    }
}
