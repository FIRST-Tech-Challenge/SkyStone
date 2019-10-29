package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

/**
 * Created by maryjane on 11/6/2018.
 * after 1st meet
 */

@TeleOp(name="cameron", group="Iterative Opmode")
@Disabled
public class cameron extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeft    = null; //rear left
    private DcMotor backRight   = null; //rear right
    private DcMotor frontLeft    = null; //front left
    private DcMotor frontRight   = null; //front right



    @Override //when init is pressed
    public void init(){
        //debugging using the phone
        telemetry.addData("status","Initialized");
        backLeft        = hardwareMap.dcMotor.get("left_drive");
        backRight       = hardwareMap.dcMotor.get("right_drive");
        frontLeft       = hardwareMap.dcMotor.get("front_left");
        frontRight      = hardwareMap.dcMotor.get("front_right");

        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override //loop after init code runs, until start is pressed
    public void init_loop() {


    }

    @Override //run once after pressing start
    public void start(){
        runtime.reset();

    }

    @Override //run loop after start code finishes
    public void loop(){
        double leftPower;
        double rightPower;

        leftPower   = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;

        if(abs(leftPower) > 0.05){
            backLeft.setPower(leftPower);
            frontLeft.setPower(leftPower);
        } else {
            backLeft.setPower(0);
            frontLeft.setPower(0);
        }

        if(abs(rightPower) > 0.05) {
            backRight.setPower(rightPower);
            frontRight.setPower(rightPower);
        } else {
            backRight.setPower(0);
            frontRight.setPower(0);
        }
    }
}
