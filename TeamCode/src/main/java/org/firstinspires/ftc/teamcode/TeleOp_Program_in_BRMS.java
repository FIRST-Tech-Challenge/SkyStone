package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOp_Program_in_BRMS")

public class TeleOp_Program_in_BRMS extends LinearOpMode {
    private DcMotor Left;
    private DcMotor Right;

    @Override
    public void runOpMode() throws InterruptedException {

        Left = hardwareMap.dcMotor.get("Left");
        Right = hardwareMap.dcMotor.get("Right");

        Left.setDirection(DcMotor.Direction.FORWARD);
        Right.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            Left.setPower(gamepad1.left_stick_y);
            Right.setPower(-gamepad1.right_stick_y);
            idle();
        }

    }


}



