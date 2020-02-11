package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;

@TeleOp (name = "Hardware_Tests")
public class hardware_tests extends OpMode {
    HardwareMap hwChss = hardwareMap;
    HardwareChassis robot;
    //Servo servo_c = null;

    @Override
    public void init() {
        robot = new HardwareChassis(hardwareMap);
        //servo_c = hardwareMap.get(Servo.class, "c_servo_hub1_port1");

        //TO DO: Neue Hardwaremap für Robot erstellen!!
    }

    @Override
    public void loop(){
        //move one servo from one psoition to another and back
        //using gamepad1.a und gamepad1.b
        if (gamepad1.a) {
            robot.servo_grab.setPosition(0.6);
        }
        else if (gamepad1.b) {
            robot.servo_grab.setPosition(0.1);
        }

        //use one motor at a time with stick y value
        //using gamepad1.right_stick_y
        else if (gamepad1.right_stick_y > 0 || gamepad1.right_stick_y < 0) {
            robot.motor_front_right.setPower(gamepad1.right_stick_y);
        }

        //use two motors in different directions with stick y value
        //using gamepad1.left_stick_y
        else if (gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0) {
            robot.motor_lift_left.setPower(gamepad1.left_stick_y);
        }

        //use one continuous servo
        //using gamepad1.x and y
        else if (gamepad1.x) {
            //servo_c.
        }

        //if nothing is pressed set all motors power 0
        else {
            robot.motor_lift_left.setPower(0);
            robot.motor_lift_right.setPower(0);
            robot.motor_front_right.setPower(0);
        }
    }

}
