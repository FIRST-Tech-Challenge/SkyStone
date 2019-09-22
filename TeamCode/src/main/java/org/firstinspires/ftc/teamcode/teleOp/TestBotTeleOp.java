package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TestBotTeleOp")
//@Disabled
public class TestBotTeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    final double calibFL = 0.75f;
    final double calibFR = 0.50f;
    final double calibBL = 1.00f;
    final double calibBR = 0.75f;

    @Override
    public void init() {
        setupMotors();


        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.clearAll();
        telemetry.update();
        driveRobot();
        telemetry.addData("Status", "Run Time: " + runtime.toString());
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
        motorFR.setPower(calibFR * power);
        motorBL.setPower(calibBR * power);
        motorBR.setPower(calibBR * -power);
    }

    public void setupMotors() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void driveRobot() {
        if (Math.abs(getRX()) < Math.abs(getLX()) || Math.abs(getRX()) < Math.abs(getLY())) {
            if (Math.abs(getLX()) < 0.1 && Math.abs(getLY()) < 0.1)
                moveForward(0);
            else if (Math.abs(getLX()) > Math.abs(getLY()))
                straifLeft(Range.clip(getLX(), -1.0, 1.0));
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

    public float getRY() {
         return -gamepad1.right_stick_y;
    }

}