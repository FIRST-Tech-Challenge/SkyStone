package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
@TeleOp(name = "MotorEncoderTest", group = "Test")
public class MotorEncoderTest extends OpMode {
    private DcMotor motor;
    private double target;
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        target = 0;
    }
    public void loop() {
        target += gamepad1.right_stick_y * 10;
        motor.setTargetPosition((int) target);
        motor.setPower(1);
        telemetry.addData("Current Position: ", motor.getCurrentPosition());
        telemetry.addData("Current Stick Position: ", gamepad1.right_stick_y);
        telemetry.addData("Current Target Position: ", motor.getTargetPosition());
        telemetry.update();
    }
}
