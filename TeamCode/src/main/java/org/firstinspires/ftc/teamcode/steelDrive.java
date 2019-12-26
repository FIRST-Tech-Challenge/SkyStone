package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "steelDrive", group = "steel")
public class steelDrive extends LinearOpMode {
    public DcMotor TL;
    public DcMotor TR;
    public DcMotor BL;
    public DcMotor BR;

    public Servo grabRectServo1;
    public Servo grabRectServo2;

    public double closePosition = 0.0;
    public double openPosition = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        TL = hardwareMap.get(DcMotor.class, "TL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        grabRectServo1 = hardwareMap.get(Servo.class, "grabRectangle1");
        grabRectServo2 = hardwareMap.get(Servo.class, "grabRectangle2");



        waitForStart();

        while (opModeIsActive()) {
            TL.setPower(-gamepad1.left_stick_y); //forward direction is -1
            BL.setPower(-gamepad1.left_stick_y); //forward direction is -1
            TR.setPower(-gamepad1.right_stick_y); //forward direction is -1
            BR.setPower(gamepad1.right_stick_y); //forward direction is 1


            if (gamepad1.right_bumper) {
                grabRectServo1.setPosition(.9);
                grabRectServo2.setPosition(.1);
            }
            else if (gamepad1.left_bumper){
                grabRectServo1.setPosition(0.1);
                grabRectServo2.setPosition(0.9);
            }

            telemetry.addData("Position of grabber: ", grabRectServo1.getPosition());
            telemetry.addData("Position of grabber: ", grabRectServo2.getPosition());
            telemetry.update();

        }
    }
}
