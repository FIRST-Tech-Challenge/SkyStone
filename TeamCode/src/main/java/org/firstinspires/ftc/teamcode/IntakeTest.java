package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class IntakeTest extends LinearOpMode {

    public void runOpMode() {
        DcMotor l = hardwareMap.dcMotor.get("0");
        l.setDirection(DcMotor.Direction.REVERSE);
        l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor r = hardwareMap.dcMotor.get("1");
        r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            while (gamepad1.a) {
                l.setPower(0.5);
                r.setPower(0.5);
            }
            while (gamepad1.b) {
                l.setPower(-0.5);
                r.setPower(-0.5);
            }
            l.setPower(0);
            r.setPower(0);
            idle();
        }
    }

}
