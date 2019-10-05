package org.firstinspires.ftc.teamcode.david_cao.testcases;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DavidCTest",group="4100")
public class BasicTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor LBMotor = hardwareMap.dcMotor.get("LB"), LFMotor = hardwareMap.dcMotor.get("LF"), RFMotor = hardwareMap.dcMotor.get("RF"), RBMotor = hardwareMap.dcMotor.get("RB");
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while(this.opModeIsActive()){

            LFMotor.setPower(gamepad1.left_stick_y);
            LBMotor.setPower(gamepad1.right_stick_y);
            RFMotor.setPower(gamepad2.left_stick_y);
            RBMotor.setPower(gamepad2.right_stick_y);
        }
    }
}
