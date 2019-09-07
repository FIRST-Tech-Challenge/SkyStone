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

        for (int i = 0; i < 100; i++){
            System.out.println(i/100+" -> "+smoothThePower(i/100));
        }

    }

    @Override
    public void loop() {

        first.setPower(gamepad1.left_stick_x);
        second.setPower(-gamepad1.left_stick_x);
        third.setPower(gamepad1.left_stick_y);
        fourth.setPower(-gamepad1.left_stick_y);

        first.setPower(gamepad1.right_stick_x);
        second.setPower(gamepad1.right_stick_x);
        third.setPower(gamepad1.left_stick_y);
        fourth.setPower(gamepad1.left_stick_y);


    }

    /**
     * a tangens hyperbolic function for a smooth gaming experience 
     * @param p
     * @return 0.390911 * tanh (6.66 * (p - 0.5))) + 0.5 )
     */
    private int smoothThePower(int p){
        return (0.390911*(java.lang.Math.tanh(6.66*(p-0.5)))+0.5)
    }
}
