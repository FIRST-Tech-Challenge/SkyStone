package org.firstinspires.ftc.teamcode.SubAssembly.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;

@TeleOp(name = "Color Test", group = "Test")
public class ColorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Color Test: ");

        /* initialize sub-assemblies
         */
        ColorControl Color = new ColorControl();
        DriveControl Drive = new DriveControl();

        GamepadWrapper egamepad1 = new GamepadWrapper(gamepad1);
        GamepadWrapper egamepad2 = new GamepadWrapper(gamepad2);

        Color.init(this);

        telemetry.update();

        //waits for that giant PLAY button to be pressed on RC
        waitForStart();


        //telling the code to run until you press that giant STOP button on RC
        while (opModeIsActive()) {

            egamepad1.updateEdge();
            egamepad2.updateEdge();

            //Color.Telemetry();


            if (egamepad1.b.pressed) {
                Color.getRed();
            } else if (egamepad1.x.pressed) {
                Color.getBlue(); }
             else if (egamepad1.a.pressed) {
                Drive.DriveUntilColor(0.5);
            }

             /*
            while (egamepad1.b.state) {
                Color.getRed();
                telemetry.addData("Red value: ", Color.redV);
                telemetry.update();
            }
            while (egamepad1.x.state) {
                Color.getBlue();
                telemetry.addData("Blue value: ", Color.blueV);
                telemetry.update();
            }

            */


            //SubAssembly.test();
            telemetry.update();

            //let the robot have a little rest, sleep is healthy
            sleep(40);
        }
    }
}
