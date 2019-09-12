package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Prototype Tester", group="Linear Opmode")
//@Disabled
public class DriveTrainPracticeCode extends LinearOpMode {

    private DcMotor leftMotor1;
    private DcMotor leftMotor2;
    private DcMotor rightMotor1;
    private DcMotor rightMotor2;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor1 = hardwareMap.dcMotor.get("left1");
        leftMotor2 = hardwareMap.dcMotor.get("left2");
        rightMotor1 = hardwareMap.dcMotor.get("right1");
        rightMotor2 = hardwareMap.dcMotor.get("right2");

        waitForStart();
        while(opModeIsActive()) {
            if(Math.abs(gamepad1.left_stick_y) < 0.15) {
                setLeftPower(0);
            }
            else {
                setLeftPower(gamepad1.left_stick_y);
            }

            if(Math.abs(gamepad1.right_stick_y) < 0.15) {
                setRightPower(0);
            }
            else {
                setRightPower(gamepad1.right_stick_y);
            }
        }


    }
    void setLeftPower(double power) {
        leftMotor1.setPower(power);
        leftMotor2.setPower(power);
    }
    void setRightPower(double power) {
        rightMotor1.setPower(power);
        rightMotor2.setPower(power);
    }
}