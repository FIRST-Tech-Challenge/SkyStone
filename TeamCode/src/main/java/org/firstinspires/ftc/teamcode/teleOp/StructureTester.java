package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "StructureTester")
public class StructureTester extends OpMode {

    private CRServo crservo = null;
    private Servo servo = null;
    double servoPos = 0.0d;

    @Override
    public void init() {
        crservo = hardwareMap.crservo.get("crservo");
        crservo.setDirection(DcMotorSimple.Direction.FORWARD);
        servo = hardwareMap.servo.get("servo");
        servo.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {
        telemetry.addData("servoPos", servoPos);
        telemetry.update();
        if (gamepad1.dpad_down) {
            crservo.setPower(0.5);
        } else if (gamepad1.dpad_up) {
            crservo.setPower(-0.5);
        } else {
            crservo.setPower(0.0);
        }

        if (gamepad1.y) {
            servoPos = Range.clip(servoPos + 0.1, 0.0, 0.5);
            servo.setPosition(servoPos);
        } else if (gamepad1.a) {
            servoPos = Range.clip(servoPos - 0.1, 0.0, 0.5);
            servo.setPosition(servoPos);
        }
    }
}
