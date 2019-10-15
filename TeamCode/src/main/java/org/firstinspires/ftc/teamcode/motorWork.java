package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "please")
public class motorWork extends LinearOpMode {

    public DcMotor armE;
    public DcMotor armR;

    @Override
    public void runOpMode() throws InterruptedException {

        armE = hardwareMap.get(DcMotor.class, "armExtender");
        armR = hardwareMap.get(DcMotor.class, "armRaiser");

        waitForStart();
        while(opModeIsActive()) {
            armE.setPower(gamepad1.right_stick_y);
            armR.setPower(gamepad1.left_stick_y);
        }
    }
}
