package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.HardwareOmnibot;

@TeleOp(name = "Omni.SMHS", group = "MateoBotics")

public class Omni extends LinearOpMode {
    HardwareOmnibot bot = new HardwareOmnibot();

    // Telemetric Operation: Manual, remote-controlled operations including movement
    // and manipulation of blocks.
    @Override
    public void runOpMode() {
        double topMotor;
        double bottomMotor;
        double rightMotor;
        double leftMotor;

        //Initialize our robot with the hardware map
        bot.init(hardwareMap);

        //Wait for robot to start...
        waitForStart();

        //Repeat while our robot's TeleOp mode is active
        while (opModeIsActive())
        {
            if (-gamepad1.left_stick_y != 0 || -gamepad1.left_stick_x != 0) {
                //Find values of left and right joysticks and find relevant power
                double horiMotor = Range.clip(-gamepad1.left_stick_x, -1.0, 1.0);
                double vertiMotor = Range.clip(-gamepad1.left_stick_y, -1.0, 1.0);
                topMotor = horiMotor;
                bottomMotor = horiMotor;
                rightMotor = vertiMotor;
                leftMotor = vertiMotor;
            }
            else {
                double rotValue = -gamepad1.right_stick_x;
                topMotor = rotValue;
                bottomMotor = rotValue;
                rightMotor = rotValue;
                leftMotor = rotValue;
            }

            bot.topDrive.setPower(topMotor);
            bot.bottomDrive.setPower(bottomMotor);
            bot.rightDrive.setPower(rightMotor);
            bot.leftDrive.setPower(leftMotor);

            //Sleep for 25 ms to keep robot from overheating...
            sleep(25);
        }
    }
}
