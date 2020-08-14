package org.firstinspires.ftc.teamcode.Skystone.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Deprecated
@TeleOp
@Disabled

public class intakeBotTeleOp extends LinearOpMode {
    DcMotor fLeft;
    DcMotor fRight;
    DcMotor bLeft;
    DcMotor bRight;
    DcMotor intakeLeft;
    DcMotor intakeRight;

    @Override
    public void runOpMode() {
        initLogic();

        waitForStart();

        telemetry.addLine("thinking inside the sphere since today");
        telemetry.update();

        while (opModeIsActive()) {
            driveLogic();
            intakeLogic();
        }
    }

    private void initLogic() {
        fLeft = getDcMotor("fLeft");
        if (fLeft != null) {
            fLeft.setDirection(DcMotor.Direction.FORWARD);
        }

        fRight = getDcMotor("fRight");
        if (fRight != null) {
            fRight.setDirection(DcMotor.Direction.REVERSE);
        }

        bLeft = getDcMotor("bLeft");
        if (bLeft != null) {
            bLeft.setDirection(DcMotor.Direction.FORWARD);
        }

        bRight = getDcMotor("bRight");
        if (bRight != null) {
            bRight.setDirection(DcMotor.Direction.REVERSE);
        }

        intakeLeft = getDcMotor("intakeLeft");
        if (intakeLeft != null) {
            intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        }

        intakeRight = getDcMotor("intakeRight");
        if (intakeRight != null) {
            intakeRight.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    private DcMotor getDcMotor(String name) {
        try {
            return hardwareMap.dcMotor.get(name);
        } catch (IllegalArgumentException exception) {
            return null;
        }
    }

    private void driveLogic() {
        double fLeftPower = -gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
        double fRightPower = -gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
        double bLeftPower = -gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;
        double bRightPower = -gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;

        fLeft.setPower(fLeftPower);
        fRight.setPower(fRightPower);
        bLeft.setPower(bLeftPower);
        bRight.setPower(bRightPower);
    }

    private void intakeLogic() {
        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        intakeRight.setPower(intakePower);
        intakeLeft.setPower(-intakePower);
    }
}
