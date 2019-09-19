package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OutOfBoundsRobot {

    // drive train motor names
    private static final String LEFT_BACK_MOTOR = "lb";
    private static final String LEFT_FRONT_MOTOR = "lf";
    private static final String RIGHT_BACK_MOTOR = "rb";
    private static final String RIGHT_FRONT_MOTOR = "rf";

    // Drive train motors
    private DcMotor leftBackMotor_;
    private DcMotor leftFrontMotor_;
    private DcMotor rightBackMotor_;
    private DcMotor rightFrontMotor_;

    LinearOpMode opMode_; // current opMode

    private void setModeForDTMotors(DcMotor.RunMode runMode){
        leftBackMotor_.setMode(runMode);
        rightBackMotor_.setMode(runMode);
        leftFrontMotor_.setMode(runMode);
        rightFrontMotor_.setMode(runMode);
        opMode_.sleep(10);
    }

    public void initDriveTrainMotors(LinearOpMode opMode){
        opMode_ = opMode;

        leftBackMotor_ = opMode_.hardwareMap.get(DcMotor.class, LEFT_BACK_MOTOR);
        leftFrontMotor_ = opMode_.hardwareMap.get(DcMotor.class, LEFT_FRONT_MOTOR);
        rightBackMotor_ = opMode_.hardwareMap.get(DcMotor.class, RIGHT_BACK_MOTOR);
        rightFrontMotor_ = opMode_.hardwareMap.get(DcMotor.class, RIGHT_FRONT_MOTOR);

        leftFrontMotor_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        opMode_.telemetry.addData("Status", "Initialized");
        opMode_.telemetry.update();
    }

    public void Move() {
        double tgtPower = -opMode_.gamepad1.left_stick_y;
        leftBackMotor_.setPower(tgtPower);
        leftFrontMotor_.setPower(tgtPower);
        rightBackMotor_.setPower(tgtPower);
        rightFrontMotor_.setPower(tgtPower);
    }
}
