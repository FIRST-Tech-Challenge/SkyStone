package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;


@TeleOp(name = "mTest", group = "Test")
public class mTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        //Harware map stuff


        //initialize Gamepads
        GamepadWrapper egamepad1 = new GamepadWrapper(gamepad1);
        GamepadWrapper egamepad2 = new GamepadWrapper(gamepad2);

        // declare motor speed variables
        double RF;
        double LF;
        double RR;
        double LR;
        // declare joystick position variables
        double X1;
        double Y1;
        double X2;
        double Y2;
        // operational constants
        double joyScale = 0.5;

        DriveControl Drive = new DriveControl();
        Drive.init(this);

        // Says what program is running and to click start
        telemetry.addLine("mTest");
        telemetry.addLine(">> Press PLAY to start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //update the gamepads
            egamepad1.updateEdge();
            egamepad2.updateEdge();

            //Ready Player One**************************************************************

            // Reset speed variables
            LF = 0;
            RF = 0;
            LR = 0;
            RR = 0;

            // Get joystick values
            Y1 = -gamepad1.left_stick_y * joyScale; // invert so up is positive
            X1 = gamepad1.left_stick_x * joyScale;
            Y2 = -gamepad1.right_stick_y * joyScale; // Y2 is not used at present
            X2 = gamepad1.right_stick_x * joyScale;

            // Forward/back movement
            LF += Y1;
            RF += Y1;
            LR += Y1;
            RR += Y1;

            // Side to side movement
            LF += X1;
            RF -= X1;
            LR -= X1;
            RR += X1;

            // Rotation movement
            LF += X2;
            RF -= X2;
            LR += X2;
            RR -= X2;

            // Send values to the motors
            //this sets speed not power because they are run using encoders
            Drive.moveMotors(LF, RF, LR, RR);

            // Send the motor speeds to the driver station
            telemetry.addData("LF", "%.3f", LF);
            telemetry.addData("RF", "%.3f", RF);
            telemetry.addData("LR", "%.3f", LR);
            telemetry.addData("RR", "%.3f", RR);

            //print the telemetry
            telemetry.update();
        }
    }
}
