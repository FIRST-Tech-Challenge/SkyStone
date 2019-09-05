package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "JonasMotor")


public class RoboTest1 extends OpMode {

    DcMotor first;
    DcMotor second;
    DcMotor third;
    DcMotor fourth;

    @Override
    public void init() {
        first = hardwareMap.dcMotor.get("first");
        second = hardwareMap.dcMotor.get("second");
        third = hardwareMap.dcMotor.get("third");
        fourth = hardwareMap.dcMotor.get("fourth");



    }

    @Override
    public void loop() {

        /*
        if(gamepad1.a) {
            first.setPower(0.5);
            second.setPower(-0.5);

        }else if(gamepad1.b){
            first.setPower(-0.5);
            second.setPower(0.5);
        }
        else {
            first.setPower(0);
            second.setPower(0);
        }
         */

        first.setPower(gamepad1.left_stick_x);
        second.setPower(-gamepad1.left_stick_x);
        third.setPower(gamepad1.left_stick_y);
        fourth.setPower(-gamepad1.left_stick_y);
        

    }
}
