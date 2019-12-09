package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;

@TeleOp(name = "FullTest")

public class FullTest extends OpMode {
    HardwareChassis robot;

    @Override
    public void init() {
        robot = new HardwareChassis(hardwareMap);
    }

    @Override
    public void loop() {
        double[] result = OmniWheel.calculate(5.0, 38, 24, -gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.1);

        telemetry.addData("Wheel FL", result[0]);
        telemetry.addData("Wheel FR", result[1]);
        telemetry.addData("Wheel RL", result[2]);
        telemetry.addData("Wheel RR", result[3]);
        telemetry.addData("R Y", gamepad1.right_stick_y);
        telemetry.addData("R X", gamepad1.right_stick_x);
        telemetry.addData("L Y", gamepad1.left_stick_y);
        telemetry.addData("L X", gamepad1.left_stick_x);

        robot.motor_front_left.setPower(result[0]);
        robot.motor_front_right.setPower(result[1]);
        robot.motor_rear_left.setPower(result[2]);
        robot.motor_rear_right.setPower(result[3]);

        robot.motor_lift_left.setPower(gamepad2.right_stick_y * 0.5);
        robot.motor_lift_right.setPower(-gamepad2.right_stick_y * 0.5);
        robot.motor_clamp.setPower(gamepad2.left_stick_x);
    }
}


