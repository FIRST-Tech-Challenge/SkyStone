package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ParadeBot")
public class ParadeBot extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftMiddle;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor rightMiddle;
    private DcMotor rightFront;

    public void runOpMode() {
        initializeMotors();
        telemetry.addData("ParadeBot: ", "Starting Program");
        telemetry.update();
        waitForStart();

        boolean slowMode = false;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                slowMode = true;
            }
            if (gamepad1.b) {
                slowMode = false;
            }
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double leftPower = Range.clip(drive + turn, -1.0, 1.0);
            double rightPower = Range.clip(drive - turn, -1.0, 1.0);
            if (slowMode) {
                setPowerLeft(leftPower / 2);
                setPowerRight(rightPower / 2);
            } else {
                setPowerLeft(leftPower);
                setPowerRight(rightPower);
            }

            telemetry.addData("SlowMode: ", slowMode);
            telemetry.update();

        }
    }

    public void initializeMotors() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftMiddle = hardwareMap.get(DcMotor.class, "leftMiddle");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightMiddle = hardwareMap.get(DcMotor.class, "rightMiddle");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMiddle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMiddle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightMiddle.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

    }

    public void setPowerLeft(double speed) {
        leftFront.setPower(speed);
        leftMiddle.setPower(speed);
        leftRear.setPower(speed);
    }

    public void setPowerRight(double speed) {
        rightRear.setPower(speed);
        rightMiddle.setPower(speed);
        rightFront.setPower(speed);
    }
}
