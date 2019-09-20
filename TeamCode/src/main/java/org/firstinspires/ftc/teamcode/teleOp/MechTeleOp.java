package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MechTeleOp")
public class MechTeleOp extends OpMode {
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    final double calibFL = 1.00f;
    final double calibFR = 1.00f;
    final double calibBL = 1.00f;
    final double calibBR = 1.00f;

    public MechTeleOp() {
        super();
    }

    @Override
    public void init() {
        setupMotors();
        resetStartTime();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("gpad1LX", gamepad1.left_stick_x);
        telemetry.addData("gpad1LY", gamepad1.left_stick_y);
        driveRobot();
        telemetry.update();
    }

    @Override
    public void stop() {
        moveForward(0.0);
    }

    public void moveForward(double power) {
        motorFL.setPower(calibFL * power);
        motorFR.setPower(calibFR * power);
        motorBL.setPower(calibBL * power);
        motorBR.setPower(calibBR * power);
    }

    public void rotateLeft(double power) {
        motorFL.setPower(calibFL * -power);
        motorFR.setPower(calibFR * power);
        motorBL.setPower(calibBL * -power);
        motorBR.setPower(calibBR * power);
    }

    public void straifLeft(double power) {
        motorFL.setPower(calibFL * -power);
        motorFR.setPower(calibFR * -power);
        motorBL.setPower(calibBR * power);
        motorBR.setPower(calibBR * power);
    }

    public void setupMotors() {
        motorFL = hardwareMap.get(DcMotor.class, "frontLeft"); // frontLeft
        motorFR = hardwareMap.get(DcMotor.class, "frontRight"); // frontRight
        motorBL = hardwareMap.get(DcMotor.class, "backLeft"); // backLeft
        motorBR = hardwareMap.get(DcMotor.class, "backRight"); // backRight

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void driveRobot() {
        if (Math.abs(getRX()) < Math.abs(getLX()) || Math.abs(getRX()) < Math.abs(getLY())) {
            if (Math.abs(getLX()) < 0.1 && Math.abs(getLY()) < 0.1)
                moveForward(0);
            else if (Math.abs(getLX()) > Math.abs(getLY()))
                straifLeft(Range.clip(-getLX(), -1.0, 1.0));
            else if (Math.abs(getLY()) > Math.abs(getLX()))
                moveForward(Range.clip(getLY(), -1.0, 1.0));
        } else {
            if (Math.abs(getRX()) < 0.1)
                moveForward(0);
            else
                rotateLeft(getRX());
        }
    }

    public float getLX() {
        return -gamepad1.left_stick_x;
    }

    public float getLY() {
        return -gamepad1.left_stick_y;
    }

    public float getRX() {
        return -gamepad1.right_stick_x;
    }
//    public float getRY() { return -gamepad1.right_stick_y; }

}
