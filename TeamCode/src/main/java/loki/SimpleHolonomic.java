package loki;

//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Simple Teleop")
public class SimpleHolonomic extends OpMode {
    private DcMotor lf, lb, rf, rb;
    BNO055IMU imu;
    double startOrient;


    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Runs based on speed instead of voltage; makes run more consistently
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    private double getHeading()
    {
        //get's current angle/way you are facing
        final Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle;

    }

    @Override
    public void start() {
        startOrient = getHeading() - Math.PI;
    }

    @Override
    public void loop() {
        final double x = gamepad1.left_stick_x;
        final double y = gamepad1.left_stick_y;
        double speed = Math.sqrt(x * x + y * y);
        //returns hypotonuse (C value in triangle)

        //calculate difference in starting angle and current angle
        double angleDiff = startOrient - getHeading();

        double robotAngle = Math.atan2(y, x) + angleDiff;
        //return angle x (next to center of circle)

        double rightX = gamepad1.right_stick_x; //reverses rotation
        //rotiation

        final double lfPow = speed * Math.sin(robotAngle - Math.PI / 4.0) - rightX;
        final double rfPow = speed * Math.cos(robotAngle - Math.PI / 4.0) + rightX;
        final double lbPow = speed * Math.cos(robotAngle - Math.PI / 4.0) - rightX;
        final double rbPow = speed * Math.sin(robotAngle - Math.PI / 4.0) + rightX;
        //determines wheel power


        rf.setPower(rfPow);
        rb.setPower(rbPow);
        lf.setPower(lfPow);
        lb.setPower(lbPow);

        telemetry.addData("Robot Angle",robotAngle*(180/Math.PI));
        telemetry.addData("heading", Math.toDegrees(getHeading()) );
        telemetry.update();
    }
}
