package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class servoTest extends LinearOpMode {
    Servo servo1;

    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.servo.get("servo1");
        waitForStart();

        while (opModeIsActive()) {
            servo1.setPosition(gamepad1.left_trigger);
        }
    }
}