package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Map;

@TeleOp(name = "ServoTester", group = "none")
public class ServoTester extends OpMode {

    private Servo arm;
    private String name;
    private Object[] servos;
    private int index;
    private int pressed;
    private double position;

    @Override
    public void init() {
        servos = hardwareMap.servo.entrySet().toArray();
        telemetry.addData("Number of servos found", servos.length);
        telemetry.update();
        if (servos.length == 0) {
            throw new IllegalArgumentException("Needs at least one servo plugged in");
        }
        index = 0;
        position = 0;
        pressed = 0;
    }

    @Override
    public void loop() {
        arm = (Servo) ((Map.Entry) servos[index]).getValue();
        name = (String) ((Map.Entry) servos[index]).getKey();
        if (gamepad1.start && !gamepad1.back && position < 1) {
            position = position + 0.001;
        }
        if (gamepad1.back && !gamepad1.start && position > 0) {
            position = position - 0.001;
        }
        arm.setPosition(position);
        telemetry.addData("Servo", name);
        telemetry.addData("Position", position);
        telemetry.update();
        if (gamepad1.dpad_up && !gamepad1.dpad_down && pressed != 1) {
            pressed = 1;
            index++;
        }
        if (gamepad1.dpad_down && !gamepad1.dpad_up && pressed != -1) {
            pressed = -1;
            index += servos.length - 1;
        }
        if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            pressed = 0;
        }
        index %= servos.length;
    }
}
