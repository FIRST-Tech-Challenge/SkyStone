package org.firstinspires.ftc.teamcode.supportops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Title: TestServo
 * Date Created: 1/12/2019
 * Date Modified: 1/12/2019
 * Author: Rahul
 * Type: Support
 * Description: This will easily allow us to test and read values from any servos easily
 */

@TeleOp
public class TestServo extends LinearOpMode {
    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException {
        // Change deviceName to test other servos
        Servo servo = hardwareMap.get(Servo.class, "servoArm");
        ElapsedTime elapsedTime = new ElapsedTime();

        waitForStart();

        elapsedTime.reset();
        while (opModeIsActive()) {
            if (elapsedTime.seconds() > .2 && (gamepad1.dpad_up || gamepad1.dpad_down)) {
                if (gamepad1.dpad_up) {
                    servo.setPosition(Range.clip(servo.getPosition() + .01, 0, 1));
                } else if (gamepad1.dpad_down) {
                    servo.setPosition(Range.clip(servo.getPosition() - .01, 0, 1));
                }
                elapsedTime.reset();
            }
            telemetry.addData("Servo pos", servo.getPosition());
            telemetry.update();
            idle();
        }
    }
}
