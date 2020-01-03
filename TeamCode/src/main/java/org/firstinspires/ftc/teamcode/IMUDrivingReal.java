package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "AdvancedDriving", group = "IMU")
public class IMUDrivingReal extends LinearOpMode {

    public DcMotor TL, TR, BL, BR;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    double power = .30;
    double correction;


    @Override
    public void runOpMode() throws InterruptedException {
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        TR = hardwareMap.get(DcMotor.class, "TR");
        TL = hardwareMap.get(DcMotor.class, "TL");

        TL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode:", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Mode:", "waiting for start");
        telemetry.addData("IMU Calibration:", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart(); //-----------------------------------------------------------------------

        sleep(1000);

        while (opModeIsActive()) {
            correction = checkDirection();

            telemetry.addData("IMU Heading:", lastAngles.firstAngle);
            telemetry.addData("Global Heading:", globalAngle);
            telemetry.addData("Correction:", correction);
            telemetry.update();

            TL.setPower(-correction);
            BL.setPower(-correction);
            TR.setPower(correction);
            BR.setPower(correction);
        }
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection() {
        double correction;


        if (getAngle() == 0)
            correction = 0;             // no adjustment.
        else if(getAngle() > 0)
            correction = -(-Math.exp(-.025 * getAngle())+1);// reverse sign of angle for correction.
        else
            correction = -Math.exp(-.025 * -getAngle())+1;

        return correction;
    }
}
