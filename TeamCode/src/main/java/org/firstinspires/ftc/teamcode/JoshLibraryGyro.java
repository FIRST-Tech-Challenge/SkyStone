package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class JoshLibraryGyro {
    // calls the hardware map
    HardwareBeep robot = null;
    // Calls the telemetry in order to send updates to the phone
    Telemetry telemetry;
    // Call this to read the last angle of the gyro
    Orientation lastAngles = new Orientation();


    // values we use throughout the program
    double globalAngle, power = .30, correction;
    double angle_variable;
    double speed;
    boolean aButton, bButton, touched;

    long lastTime;
    double Input, Output, Setpoint;
    double errSum, lastErr;
    double kp, ki, kd;

    /**
     * The hardware class needs to be initialized before this function is called
     */
    public void init(HardwareBeep myRobot, Telemetry myTelemetry) {
        // initializes all the programs we are pulling for this Library
        robot = myRobot;
        telemetry = myTelemetry;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Sets parameters for gyro
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // initializes the gyro in the Rev IMU
        robot.imu.initialize(parameters);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        // We completely reset the gyro angle
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);

        //
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }


    public void ComputePID() {
        long now = System.currentTimeMillis();
        double timeChange = (double) (now - lastTime);
        double error = Setpoint - Input;
        errSum += (error * timeChange);
        double dErr = (error - lastErr);

        Output = kp * error + ki * errSum + kd * dErr;
        lastErr = error;
        lastTime = now;

    }

    /** This method sets the PID values
     * @param Kp
     * @param Ki
     * @param Kd
     */
    public void SetTunings(double Kp, double Ki, double Kd) {
        // Add values for PID algorithm
        // The P value stands for Proportional
        kp = Kp;
        // I stands for Integral
        ki = Ki;
        // D stands for Derivative
        kd = Kd;
    }


    /** This is the turn Gyro method that call in other programs to turn the robot to a certain
     * degree.
     * @param targetHeading
     * @return
     */
    public double turnGyro(float targetHeading) {
        // Drive Gain value is used to track the gyros position
        double DRIVEGAIN = 1;
        //
        double TOLERANCE = .5;
        double timer = 0;
        double currentHeading = 0.0;
        long startTime = 0;
        double polarity = 1;
        polarity = targetHeading > 0 ? 1 : -1;

        resetAngle();
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        startTime = System.currentTimeMillis();
        currentHeading = getAngle();
        SetTunings(.02, 0, 0.1);

        Setpoint = targetHeading;
        Input = getAngle();
        telemetry.addData("Current Pos ", currentHeading);
        telemetry.addData("Setpoint ", Setpoint);
        telemetry.addData("Input ", Input);
        telemetry.update();

        Output *= polarity;

        do {

            ComputePID();
            robot.leftFront.setPower(Output);
            robot.leftBack.setPower(Output);
            robot.rightFront.setPower(-Output);
            robot.rightBack.setPower(-Output);
            timer++;
            Input = getAngle();
            telemetry.addData("curHeading", Input);
            telemetry.addData("tarHeading", Setpoint);
            telemetry.update();
        }
        while ((Math.abs(Input - Setpoint) > TOLERANCE) || (System.currentTimeMillis() < (startTime + 1050)));


        telemetry.addData("curHeading", Input);
        telemetry.addData("tarHeading", Setpoint);
        telemetry.addData("leftPwr", -Output);
        telemetry.addData("rightPwr", Output);
        telemetry.addData("DRIVEGAIN", DRIVEGAIN);
        telemetry.update();

        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        return Input;

    }

}