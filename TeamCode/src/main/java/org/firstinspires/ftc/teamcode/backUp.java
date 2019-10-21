package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "backup")
public class backUp extends LinearOpMode {

    public DcMotor armE;

    @Override
    public void runOpMode() throws InterruptedException {
        armE = hardwareMap.get(DcMotor.class, "armExtender");
        waitForStart();
        while (opModeIsActive()) {
            armE.setPower(gamepad1.right_stick_y);
        }
    }
}
