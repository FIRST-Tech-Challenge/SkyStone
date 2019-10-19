/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.util.RobotLog;



//import lines go here. This is just for the program and not for the robot.

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Nerd PID Drive", group="Linear Opmode")
//@Disabled
public class NerdPIDrive extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu = null;   // Gyro device

    double integral = 0; // Variable to hold cumulative integral value.


    //Initial Speed for Robot to run

    double speed = 0.8;

    //Gains

    double kP = 0.6; // Gain for Proportional drive
    double kI = 0.4; //Gain for Integral drive

    NerdPIDCalculator straightDrivePIDCalculator = new NerdPIDCalculator("DriveStraightPID", 0.2, 0.8, 0.8);
    NerdPIDCalculator turnPIDCalculator = new NerdPIDCalculator("TurnPID", 0.4, 0.2, 0.2);
    static final double HEADING_THRESHOLD = 1;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        straightDrivePIDCalculator.setTarget(0.0);

        waitForStart();

        RobotLog.d("NerdPIDrive - Starting Straight Drive");
        PIDrive(24.0);
        turnPIDCalculator.setTarget(90.0);
        PIDTurn(0.5);

        while (opModeIsActive())  {

        }


    }

    public double getGyroError(double targetAngle) {
        Orientation angles;
        double error;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        error = targetAngle - angles.firstAngle;
        return error;
    }

    public double getProportion(double error, double gain) {
        double P;
        P = error * gain;
        return P;
    }

    public double getIntegral(double error) {
        integral = integral + (error * runtime.seconds());
        return integral;
    }

    public int inchesToTicks(double wheel_diameter, double inches) {
        double ticks;
        double circum;
        circum = wheel_diameter * 3.14;
        ticks = (1120 / circum) * inches; // (COUNTS_PER_MOTOR_REV )/(WHEEL_CIRCUMFERENCE) * distance;
        return (int) Math.round(ticks);

    }

    public void PIDrive(double distance) {

        double max;
        double leftSpeed;
        double rightSpeed;

        int ticksToMove = 0;

        double pid = 0.0;

        runtime.reset();
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        ticksToMove = inchesToTicks(4.0, distance);

        leftMotor.setTargetPosition(ticksToMove);
        rightMotor.setTargetPosition(ticksToMove);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);


        while (leftMotor.isBusy() && rightMotor.isBusy()) {
            //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //measuredAngle= angles.firstAngle;
     /*       double gyroError;
            gyroError = getGyroError(0.0);

            double Proportional;

            Proportional = getProportion(gyroError, kP);

            double integralOutput = 0;
            integralOutput = getIntegral(gyroError);

    */
            pid = straightDrivePIDCalculator.getOutput(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            leftSpeed = speed - pid;
            rightSpeed = speed + pid;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0) {
                leftSpeed /= max;
                rightSpeed /= max;
            }


            leftMotor.setPower(leftSpeed);
            rightMotor.setPower(rightSpeed);
            //////////////////////////////////////////////////////////////////////////////////

            telemetry.addData("leftMotorTicks", leftMotor.getCurrentPosition());
            telemetry.addData("rigthMotorTicks", rightMotor.getCurrentPosition());
            telemetry.update();


        }
    }

    public void PIDTurn(double turnspeed) {

        Orientation angles;
        double pidvalue;
        double max;
        double leftSpeed, rightSpeed=0.0;
        runtime.reset();

        //while (opModeIsActive() && !onTarget(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)) {
          //  speed = Range.clip(Math.abs(turnspeed), 0.0, 1.0);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (opModeIsActive() && !onTarget(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)) { //&& leftMotor.isBusy() && rightMotor.isBusy()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                pidvalue = turnPIDCalculator.getOutput(angles.firstAngle);

                leftSpeed = turnspeed - pidvalue;
                rightSpeed = turnspeed + pidvalue;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftMotor.setPower(leftSpeed);
                rightMotor.setPower(rightSpeed);
                //////////////////////////////////////////////////////////////////////////////////

                RobotLog.d("PIDCaller - : current Angle %f, pidValue %f, leftSpeed %f, rightSpeed %f", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, pidvalue, leftSpeed, rightSpeed);
                telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();

            }

            leftMotor.setPower(0);
            rightMotor.setPower(0);

        //}


    }

    boolean onTarget(double angle) {
        double error;
        boolean onTarget = false;


        // determine turn power based on +/- error
        error = turnPIDCalculator.getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {

            onTarget = true;
        }

        return onTarget;
    }
}

