package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareOmniTest;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;

@TeleOp(name = "OmniCalc")

public class OmniCalcTest extends OpMode {
    HardwareOmniTest hwMap;

    @Override
    public void init() {
        hwMap = new HardwareOmniTest(hardwareMap);
    }

    @Override
    public void loop() {
        double[] result = OmniWheel.calculate(5.0, 38, 24, -gamepad1.left_stick_y*0.5, gamepad1.left_stick_x*0.5, gamepad1.right_stick_x*0.25);

        telemetry.addData("Wheel A", result[0]);
        telemetry.addData("Wheel B", result[1]);
        telemetry.addData("Wheel C", result[2]);
        telemetry.addData("Wheel D", result[3]);
        telemetry.addData("R Y", -gamepad1.right_stick_y);
        telemetry.addData("R X", gamepad1.right_stick_x);
        telemetry.addData("L Y", -gamepad1.left_stick_y);
        telemetry.addData("L X", gamepad1.left_stick_x);
        hwMap.motor_front_left.setPower(result[0]);
        hwMap.motor_front_right.setPower(result[1]);
        hwMap.motor_rear_left.setPower(result[2]);
        hwMap.motor_rear_right.setPower(result[3]);
    }
}
