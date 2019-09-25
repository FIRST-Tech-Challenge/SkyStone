package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.HardwareDummybot;

@TeleOp(name = "Kamikaze.SMHS", group = "MateoBotics")

public class Kamikaze extends LinearOpMode {
    HardwareDummybot bot = new HardwareDummybot();

    // Telemetric Operation: Manual, remote-controlled operations including movement
    // and manipulation of blocks.
    @Override
    public void runOpMode() {
        double leftMotor;
        double rightMotor;

        //Initialize our robot with the hardware map
        bot.init(hardwareMap);

        //Wait for robot to start...
        waitForStart();

        //Repeat while our robot's TeleOp mode is active
        while (opModeIsActive())
        {
            //Find values of left and right joysticks and find relevant power
            leftMotor = Range.clip(-gamepad1.left_stick_y, -1.0, 1.0);
            rightMotor =  Range.clip(-gamepad1.right_stick_y, -1.0, 1.0);

            //Set power of left and right motors
            bot.leftDrive.setPower(leftMotor);
            bot.rightDrive.setPower(rightMotor);

            //Set telemetry data for future debugging
            telemetry.addData("Left power:", leftMotor);
            telemetry.addData("Right power:", rightMotor);
            telemetry.update();

            //Sleep for 25 ms to keep robot from overheating...
            sleep(25);
        }
    }
}
