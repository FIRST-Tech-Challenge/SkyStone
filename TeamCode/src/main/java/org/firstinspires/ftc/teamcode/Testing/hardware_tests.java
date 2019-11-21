package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassisGyro;

@TeleOp (name = "hardware_tests")
public class hardware_tests extends OpMode {
    Servo servo_hub1_port0 = null;
    DcMotor motor_hub1_port0 = null;
    DcMotor motor_hub1_port1 = null;

    @Override
    public void init() {
        servo_hub1_port0 = hardwareMap.get(Servo.class, "servo_hub1_port0");
        motor_hub1_port0 = hardwareMap.get(DcMotor.class, "motor_hub1_port0");
        motor_hub1_port1 = hardwareMap.get(DcMotor.class, "motor_hub1_port1");

        //TO DO: Neue Hardwaremap für Robot erstellen!!
    }

    @Override
    public void loop(){
        //move one servo from one psoition to another and back
        //using gamepad1.a und gamepad1.b
        if (gamepad1.a) {
            servo_hub1_port0.setPosition(0.6);
        }
        else if (gamepad1.b) {
            servo_hub1_port0.setPosition(0.1);
        }

        //use one motor at a time with stick y value
        //using gamepad1.right_stick_y
        else if (gamepad1.right_stick_y > 0 || gamepad1.right_stick_y < 0) {
            motor_hub1_port0.setPower(gamepad1.right_stick_y);
        }

        //use two motors in different directions with stick y value
        //using gamepad1.left_stick_y
        else if (gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0) {
            motor_hub1_port0.setPower(gamepad1.left_stick_y);
            motor_hub1_port1.setPower(-gamepad1.left_stick_y);
        }

        //if nothing is pressed set all motors power 0
        else {
            motor_hub1_port0.setPower(0);
            motor_hub1_port1.setPower(0);
        }
    }

}
