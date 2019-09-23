package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTester", group = "none")
public class ServoTester extends OpMode {

    private Servo arm;
    private double position;

    @Override
    public void init() {
        arm = hardwareMap.get(Servo.class, "arm");
        position = 0;
    }

    @Override
    public void loop() {
        if (gamepad1.start && !gamepad1.back && position < 1) {
            position = position + 0.001;
        }
        if (gamepad1.back && !gamepad1.start && position > 1) {
            position = position - 0.001;
        }
        arm.setPosition(position);
    }
}
