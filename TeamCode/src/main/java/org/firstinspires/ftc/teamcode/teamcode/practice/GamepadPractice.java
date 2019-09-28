package org.firstinspires.ftc.teamcode.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "Gamepad Practice Drive", group="Training")
public class GamepadPractice extends OpMode {

    private DcMotor leftWheel;
    private DcMotor rightWheel;


    private Servo leftClaw;
    private Servo rightClaw;

    private double leftWheelPower = 0.0;
    private double rightWheelPower = 0.0;
    //const

    private static final int CLOSE = 1;
    private static final int OPEN = 1;


    @Override
    public void init() {

        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        leftWheel = hardwareMap.get(DcMotor.class, "left_wheel");
        rightWheel = hardwareMap.get(DcMotor.class, "right_wheel");

        leftWheel.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        //tank style driving
        leftWheelPower = gamepad1.left_stick_y;
        rightWheelPower = gamepad1.right_stick_y;

        leftWheel.setPower(leftWheelPower);
        rightWheel.setPower(rightWheelPower);

        //game pad x button
        if(gamepad1.x)
        {
            leftClaw.setPosition(CLOSE);
            rightClaw.setPosition(CLOSE);
        }
        else
        {
            leftClaw.setPosition(OPEN);
            rightClaw.setPosition(OPEN);
        }

        //game pad y button



    }
}
