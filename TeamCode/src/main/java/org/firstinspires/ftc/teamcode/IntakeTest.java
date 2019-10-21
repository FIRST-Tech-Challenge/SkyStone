package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class IntakeTest extends LinearOpMode {

    private DcMotor l = null;
    private DcMotor r = null;

    public void runOpMode() {
        DcMotor l = hardwareMap.dcMotor.get("0");
        l.setDirection(DcMotor.Direction.REVERSE);
        l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor r = hardwareMap.dcMotor.get("1");
        r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            while (gamepad1.a) {
                setPower(0.5);
            }
            while (gamepad1.b) {
                setPower(-0.5);
            }
            setPower(0);
            idle();
        }

    }

    private void setPower(double power) {
        l.setPower(power);
        r.setPower(power);
    }

}
