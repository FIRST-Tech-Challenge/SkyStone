package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

@TeleOp

public class MecanumTeleOp extends LinearOpMode {
    public void runOpMode() {

        //358-2019 Declare Motors
        DcMotor fL = hardwareMap.dcMotor.get("fL"); //0
        fL.setDirection(DcMotor.Direction.REVERSE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor fR = hardwareMap.dcMotor.get("fR"); //2
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor bL = hardwareMap.dcMotor.get("bL"); //1
        bL.setDirection(DcMotor.Direction.REVERSE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor bR = hardwareMap.dcMotor.get("bR"); //3
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double SCALE = 0.4;

        waitForStart();
        while (opModeIsActive()) {

            //358-2019 Drive code :) ///////////////////////////////////////////////////////////////
            //Defining drive, strafe, and rotation power.
            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            //Defining the motor power distribution.
            double flPower = drive - strafe + rotate;
            double blPower = drive + strafe + rotate;
            double frPower = drive + strafe - rotate;
            double brPower = drive - strafe - rotate;

            double joyStick = Range.clip(max(magnitudeLeftStick(gamepad1), abs(rotate)), -1, 1);
            double POWER = -1 * joyStick * abs(joyStick);
            double maxPower = findMax(abs(flPower), abs(blPower), abs(frPower), abs(brPower));

            //Sets the power for all the drive motors.
            fL.setPower(-(POWER * flPower / maxPower) * SCALE);
            bL.setPower(-(POWER * blPower / maxPower) * SCALE);
            fR.setPower(-(POWER * frPower / maxPower) * SCALE);
            bR.setPower(-(POWER * brPower / maxPower) * SCALE);
            //358-2019 Drive code :)////////////////////////////////////////////////////////////////

            idle();
        }
    }

    /**
     * This 358-2019 function finds the max value given 4 values.
     *
     * @param d1 value 1
     * @param d2 value 2
     * @param d3 value 3
     * @param d4 value 4
     * @return max value among all four values
     */
    private Double findMax(Double d1, Double d2, Double d3, Double d4) {
        return max(max(d1, d2), max(d3, d4));
    }

    /**
     * This 358-2019 function finds the magnitude of the left stick of a game pad.
     *
     * @param gamepad game pad
     * @return (double) magnitude of the left stick of game pad
     */
    private Double magnitudeLeftStick(Gamepad gamepad) {
        return sqrt(pow(gamepad.left_stick_x, 2) + pow(gamepad.left_stick_y, 2));
    }
}
