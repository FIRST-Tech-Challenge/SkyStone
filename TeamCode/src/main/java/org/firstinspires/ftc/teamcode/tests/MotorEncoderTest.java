package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
@TeleOp(name = "MotorEncoderTest", group = "Test")
public class MotorEncoderTest extends OpMode {
    private DcMotor motor;
    private int target;
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void loop() {
        target += gamepad1.right_stick_y;
        motor.setTargetPosition(target);
        telemetry.addData("Current position: ", motor.getCurrentPosition());
        telemetry.update();
    }
}
