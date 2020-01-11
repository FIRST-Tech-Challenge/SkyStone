package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;

@Disabled

//@TeleOp(name = "WheelTest")

public class WheelTest extends OpMode {
    HardwareChassis robot;

    @Override
    public void init() {
        robot = new HardwareChassis(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.y) {
            robot.motor_front_left.setPower(-1);
        } else {
            robot.motor_front_left.setPower(0);
        }

        if(gamepad1.b) {
            robot.motor_front_right.setPower(1);
        } else {
            robot.motor_front_right.setPower(0);
        }

        if(gamepad1.x) {
            robot.motor_rear_left.setPower(-1);
        } else {
            robot.motor_rear_left.setPower(0);
        }

        if(gamepad1.a) {
            robot.motor_rear_right.setPower(1);
        } else {
            robot.motor_rear_right.setPower(0);
        }
    }
}