package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    public void runOpMode() {
        DcMotor l = hardwareMap.dcMotor.get("0");
        l.setDirection(DcMotor.Direction.REVERSE);
        l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor r = hardwareMap.dcMotor.get("2");
        r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor l1 = hardwareMap.dcMotor.get("1");
        l1.setDirection(DcMotor.Direction.REVERSE);
        l1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor r1 = hardwareMap.dcMotor.get("3");
        r1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor il = hardwareMap.dcMotor.get("4");
        l.setDirection(DcMotor.Direction.REVERSE);
        l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor ir = hardwareMap.dcMotor.get("5");
        r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double s = 0.65;
        waitForStart();
        while (opModeIsActive()) {
            double v = -gamepad1.left_stick_y;
            double v1 = -gamepad1.left_stick_x;
            double x = gamepad1.right_stick_x;
            double v2 = v - v1 + x;
            double v3 = v + v1 + x;
            double v4 = v + v1 - x;
            double v5 = v - v1 - x;
            double v6 = Range.clip(max(m(gamepad1), abs(x)), -1, 1);
            double v7 = -1 * v6 * abs(v6);
            double f = f(abs(v2), abs(v3), abs(v4), abs(v5));
            l.setPower(-(v7 * v2 / f) * s);
            l1.setPower(-(v7 * v3 / f) * s);
            r.setPower(-(v7 * v4 / f) * s);
            r1.setPower(-(v7 * v5 / f) * s);
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

    private Double f(Double d1, Double d2, Double d3, Double d4) {
        return max(max(d1, d2), max(d3, d4));
    }

    private Double m(Gamepad gamepad) {
        return sqrt(pow(gamepad.left_stick_x, 2) + pow(gamepad.left_stick_y, 2));
    }
}