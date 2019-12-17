package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.reflect.Type;

@TeleOp(name = "manualOp", group = "DeadLine")
public class BasicTeleOp extends LinearOpMode {

    public DcMotor BL;
    public DcMotor BR;
    public DcMotor TR;
    public DcMotor TL;

    public DcMotor armEx;
    public DcMotor armRz;


    final double strafeSpeed = 0.5;
    final double closePosition = 1.0;
    final double openPosition = 0.0;

    private int upperBound = 1600;
    private int lowerBound = 0;

    public Servo grabServo;
    public Servo rotateServo;

    @Override
    public void runOpMode() throws InterruptedException {
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        TR = hardwareMap.get(DcMotor.class, "TR");
        TL = hardwareMap.get(DcMotor.class, "TL");
        armEx = hardwareMap.get(DcMotor.class, "armEx");
        armEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRz = hardwareMap.get(DcMotor.class, "armRz");
        armRz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        grabServo = hardwareMap.get(Servo.class, "grabServo");
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");

        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        TL.setDirection(DcMotorSimple.Direction.FORWARD);
        TR.setDirection(DcMotorSimple.Direction.FORWARD);



        waitForStart();

        while(opModeIsActive()){

            telemetry.addData("Running?", opModeIsActive());
            telemetry.addData("armEx: ", armEx.getCurrentPosition());
            telemetry.addData("armRz", armRz.getCurrentPosition());
            telemetry.update();

            TL.setPower(-gamepad1.right_stick_y);
            BL.setPower(-gamepad1.right_stick_y);
            TR.setPower(gamepad1.left_stick_y);
            BR.setPower(gamepad1.left_stick_y);

            if (gamepad1.dpad_left){
                TL.setPower(-strafeSpeed);
                BL.setPower(strafeSpeed);
                BR.setPower(-strafeSpeed);
                TR.setPower(strafeSpeed);
            }
            else if (gamepad1.dpad_right) {
                TL.setPower(strafeSpeed);
                BL.setPower(strafeSpeed);
                BR.setPower(strafeSpeed);
                TR.setPower(strafeSpeed);
            }

            if (armEx.getCurrentPosition()>upperBound) {
                armEx.setPower(-1.0);
            }
            else if (armEx.getCurrentPosition()<lowerBound) {
                armEx.setPower(1.0);
            }
            else {
                armEx.setPower(gamepad2.left_stick_y);
            }

            armEx.setPower(gamepad2.left_stick_y);
            //Set loop for controller to stop at minimum limit and maximum limit
            armRz.setPower(gamepad2.right_stick_y);
            //Set loop for controller to stop at minimum limit and maximum limit

            if (gamepad2.left_trigger>0){
                grabServo.setPosition(openPosition);
            }
            else if (gamepad2.right_trigger>0){
                grabServo.setPosition(closePosition);
            }

            if (gamepad2.left_bumper){
                rotateServo.setPosition(1);
            }
            else if (gamepad2.right_bumper) {
                rotateServo.setPosition(0);
            }
        }
    }
}
