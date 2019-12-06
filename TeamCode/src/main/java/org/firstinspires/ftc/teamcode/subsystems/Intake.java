package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.subsystemutils.Subsystem;

import static android.os.SystemClock.sleep;

public class Intake implements Subsystem {

    private Gamepad controller;
    private Servo bootServo;
    private DcMotor leftIntake, rightIntake;


    public Intake(Gamepad controller, Servo bootServo, DcMotor leftIntake, DcMotor rightIntake) {
        this.controller = controller;
        this.bootServo = bootServo;
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
    }

    public void init() {
        bootServo.setPosition(0.0);
        leftIntake.setPower(0.5);
        rightIntake.setPower(0.5);
        sleep(500);
        leftIntake.setPower(0.0);
        rightIntake.setPower(0.0);
    }

    public void update() {
        if (controller.right_bumper) {
            leftIntake.setPower(-.25);
            rightIntake.setPower(.25);
        } else if (controller.left_bumper) {
            leftIntake.setPower(1.0);
            rightIntake.setPower(-1.0);
        } else {
            rightIntake.setPower(0.0);
            rightIntake.setPower(0.0);
        }

        if (controller.y) {
            bootServo.setPosition(180.0);
        }

        if (controller.a) {
            bootServo.setPosition(0.0);
        }
    }
}
