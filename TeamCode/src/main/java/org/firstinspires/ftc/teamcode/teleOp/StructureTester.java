package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "StructureTester")
public class StructureTester extends OpMode {

    private Servo hand = null;
    double handPos = 0.0d;

    @Override
    public void init() {
        hand = hardwareMap.servo.get("hand");
    }

    @Override
    public void loop() {
        telemetry.addData("handPos", handPos);
        telemetry.update();
        if (gamepad1.dpad_down) {
            handPos = Range.clip(handPos + 0.1, 0.0, 0.5);
            hand.setPosition(handPos);
        } else if (gamepad1.dpad_up) {
            handPos = Range.clip(handPos - 0.1, 0.0, 0.5);
            hand.setPosition(handPos);
        }
    }
}
