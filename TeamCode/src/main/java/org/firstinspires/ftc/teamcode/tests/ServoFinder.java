package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Finder", group = "Test")
public class ServoFinder extends OpMode {

    public Servo pivot;
    public Servo elbow;
    public Servo wrist;
    public double speed = 0.001;

    public void init() {
        pivot = hardwareMap.get(Servo.class, "PIVOT");
        elbow = hardwareMap.get(Servo.class, "ELBOW");
        wrist = hardwareMap.get(Servo.class, "WRIST");

    }
    public void loop() {
        if (gamepad1.a) {
            pivot.setPosition(pivot.getPosition() + speed);
        }

        if (gamepad1.b) {
            elbow.setPosition(elbow.getPosition() + speed);
        }
        if (gamepad1.y) {
            wrist.setPosition(wrist.getPosition() + speed);
        }

        if (gamepad1.dpad_down) {
            pivot.setPosition(pivot.getPosition() - speed);
        }
        if (gamepad1.dpad_right) {
            elbow.setPosition(elbow.getPosition() - speed);
        }
        if (gamepad1.dpad_up) {
            wrist.setPosition(wrist.getPosition() - speed);
        }

        telemetry.addData("Pivot: ", pivot.getPosition());
        telemetry.addData("Elbow: ", elbow.getPosition());
        telemetry.addData("Wrist: ", wrist.getPosition());
        telemetry.update();
    }
}
