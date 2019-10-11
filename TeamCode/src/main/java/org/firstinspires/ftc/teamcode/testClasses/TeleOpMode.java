package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpMode", group = "Teleop")
public class TeleOpMode extends LinearOpMode {
    public void runOpMode() {
        //IMU
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(getParams());
        //Servos
        Servo hook = hardwareMap.servo.get("hook");
        //Motors
        DcMotor frontLeftDrive = hardwareMap.dcMotor.get("front_left_drive");
        DcMotor frontRightDrive = hardwareMap.dcMotor.get("front_right_drive");
        DcMotor backLeftDrive = hardwareMap.dcMotor.get("back_left_drive");
        DcMotor backRightDrive = hardwareMap.dcMotor.get("back_right_drive");
        //DcMotor leftIntake = hardwareMap.dcMotor.get("left_intake");
        //DcMotor rightIntake = hardwareMap.dcMotor.get("right_intake");
        telemetry.addData("Init", "v:1.0");
        waitForStart();

        while (opModeIsActive()) {
            //Unit text
            if (gamepad1.a)
                frontLeftDrive.setPower(1);
            if (gamepad1.b)
                frontRightDrive.setPower(1);
            if (gamepad1.x)
                backLeftDrive.setPower(1);
            if (gamepad1.y)
                backRightDrive.setPower(1);
            //Hook test
            if (gamepad1.a == true) {
                hook.setPosition(hook.getPosition() + .1);
            } else if (gamepad1.b == true) {
                hook.setPosition(hook.getPosition() - .1);
            }

             /*
            //Intake test
            if (gamepad1.x == true) {
                leftIntake.setPower(1);
                rightIntake.setPower(-1);
            }
            else if (gamepad1.y == true) {
                leftIntake.setPower(-1);
                rightIntake.setPower(1);
            }
            else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }
            */

            //Chassis test
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;//Y on controll stick is inverted
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;
            frontLeftDrive.setPower(v1);
            frontRightDrive.setPower(-v2);//Left hand rule makes right side neg
            backLeftDrive.setPower(v3);
            backRightDrive.setPower(-v4);//Left hand rule makes right side neg
            telemetry.addData("right stick: ",gamepad1.left_stick_y);
            telemetry.addData("robot angle: ", robotAngle);
            telemetry.addData("hook position: ", hook.getPosition());
            telemetry.update();
        }
    }

    public BNO055IMU.Parameters getParams() {
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        return parameters;
    }
}