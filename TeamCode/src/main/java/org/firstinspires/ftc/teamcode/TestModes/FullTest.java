package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;

@TeleOp(name = "FullTest")

public class FullTest extends OpMode {
    HardwareChassis robot;
    double smootingValue;

    @Override
    public void init() {
        robot = new HardwareChassis(hardwareMap);
        smootingValue = -0.5;
    }

    @Override
    public void loop() {
        double[] result = OmniWheel.calculate(
                5.0,
                38,
                24,
                GeneralTools.calculateControllerSmooting(-gamepad1.left_stick_y, smootingValue)*0.5,
                GeneralTools.calculateControllerSmooting(gamepad1.left_stick_x, smootingValue)*0.5,
                GeneralTools.calculateControllerSmooting(gamepad1.right_stick_x, smootingValue)*-0.02);

        /*
        double[] result = OmniWheel.calculate(
                5.0,
                38,
                24,
                -gamepad1.left_stick_y*0.5,
                gamepad1.left_stick_x*0.5,
                gamepad1.right_stick_x*0.05);
         */

        if (gamepad1.right_bumper) {
            smootingValue = smootingValue+0.05;
            while (gamepad1.right_bumper){}
        }
        if (gamepad1.left_bumper) {
            smootingValue = smootingValue-0.05;
            while (gamepad1.left_bumper){}
        }

        telemetry.addData("smootingValue", smootingValue);

        robot.motor_front_left.setPower(result[0]);
        robot.motor_front_right.setPower(result[1]);
        robot.motor_rear_left.setPower(result[2]);
        robot.motor_rear_right.setPower(result[3]);

        robot.motor_lift_left.setPower(gamepad2.right_stick_y*0.5);
        robot.motor_lift_right.setPower(gamepad2.right_stick_y*0.5);
        robot.motor_clamp.setPower(gamepad2.left_stick_x);
    }
}