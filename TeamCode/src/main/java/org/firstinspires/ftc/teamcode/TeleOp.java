package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    private DcMotor left;
    private DcMotor right;
    private Servo claw;
    private boolean servoBoolean;
    private double servoTime = System.currentTimeMillis();
    public void runOpMode() throws InterruptedException{
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        claw = hardwareMap.servo.get("claw");
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()){
            drive(gamepad1.left_stick_y);
            rotate(gamepad1.right_stick_x);
            servoControl();
            idle();
        }
    }
    public void drive(double power){
        left.setPower(power);
        right.setPower(power);
    }
    public void rotate(double power){
        left.setPower(power);
        right.setPower(-power);
    }
    public void clamp(){
        claw.setPosition(1);
    }
    public void release(){
        claw.setPosition(0);
    }
    public void servoControl(){
        if(gamepad1.a && !servoBoolean && System.currentTimeMillis() > servoTime + 300){
            clamp();
            servoBoolean = true;
            servoTime = System.currentTimeMillis();
        }
        else if(gamepad1.a && servoBoolean && System.currentTimeMillis() > servoTime + 300){
            release();
            servoBoolean = false;
            servoTime = System.currentTimeMillis();
        }
    }
}
