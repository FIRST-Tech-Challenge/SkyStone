package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Robot: Teleop", group="Steve")
public class RobotTelop extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot      = new RobotHardware();   // Use a Pushbot's hardware
    int counter       = 0;                     // Servo mid position

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        if (robot.leftDrive == null) {
            telemetry.addData("Warning", "Motor: left_drive not plugged in");    //
        } else {
            telemetry.addData("Status", "Motor: left_drive identified");    //
        }

        if (robot.rightDrive == null) {
            telemetry.addData("Warning", "Motor: right_drive not plugged in");    //
        } else {
            telemetry.addData("Status", "Motor: right_drive identified");    //
        }

        if (robot.leverArm == null) {
            telemetry.addData("Warning", "Motor: lever_arm not plugged in");    //
        } else {
            telemetry.addData("Status", "Motor: lever_arm identified");    //
        }

        if (robot.clampRotator == null) {
            telemetry.addData("Warning", "Servo: clamp_rotator not plugged in");    //
        } else {
            telemetry.addData("Status", "Servo: clamp_rotator identified");    //
        }

        if (robot.clamp == null) {
            telemetry.addData("Warning", "Servo: clamp not plugged in");    //
        } else {
            telemetry.addData("Status", "Servo: clamp identified");    //
        }
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            telemetry.addData("We be counting",  "Counter = %7d", counter);
            telemetry.update();

            // Pace this loop so servo speed is reasonable.
            sleep(1000);

            counter += 1;

        }
    }
}