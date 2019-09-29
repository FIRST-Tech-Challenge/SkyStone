package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MyOpMode extends LinearOpMode {
    OutOfBoundsRobot robot = new OutOfBoundsRobot();

    @Override
    public void runOpMode() {
        try {
            robot.initDriveTrainMotors(this);

            telemetry.addData("Status", "Hardware Initialized");
            telemetry.update();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // run until the end of the match (driver presses STOP)

            while (opModeIsActive()) {
                robot.loop();
                telemetry.addData("Status", "Running");
                telemetry.update();
            }
        } catch (IllegalArgumentException e) {
            telemetry.addData("Status", e.getMessage());
            telemetry.update();
        }
    }
}