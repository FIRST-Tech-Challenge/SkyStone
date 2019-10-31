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
    private double[] positions;

    @Override
    public void init() {
        servos = hardwareMap.servo.entrySet().toArray();
        telemetry.addData("Number of servos found", servos.length);
        telemetry.addLine("Start to move forward, back to move in reverse.");
        telemetry.addLine("Dpad up and down to switch servos.");
        telemetry.update();
        if (servos.length == 0) {
            throw new IllegalArgumentException("Needs at least one servo plugged in");
        }
        index = 0;
        positions = new double[servos.length];
        pressed = 0;
    }

    @Override
    public void loop() {
        arm = (Servo) ((Map.Entry) servos[index]).getValue();
        name = (String) ((Map.Entry) servos[index]).getKey();
        if (gamepad1.start && !gamepad1.back && positions[index] < 1) {
            positions[index] += 0.001;
        }
        if (gamepad1.back && !gamepad1.start && positions[index] > 0) {
            positions[index] -= 0.001;
        }
        arm.setPosition(positions[index]);
        telemetry.addData("Servo", name);
        telemetry.addData("Position", positions[index]);
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
