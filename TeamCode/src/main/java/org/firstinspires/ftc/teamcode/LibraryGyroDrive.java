package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * This is our custom library that we use to drive forward. The point of using a gyro sensor is to
 * drive straight using it reaches a certain position. If the robot runs over an object or hits
 * something it will take the error of the robot and correct it to get back on track and continue
 * driving forward
 */
public class LibraryGyroDrive {

    // This value is to read the robots current heading
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    // This P Coefficient is the value that corrects the robot in relation to the error of the robots
    // current heading
    static final double P_DRIVE_COEFF = 0.05;     // Larger is more responsive, but also less stable
    // a constant speed
    public double speed = .6;
    // Calls the Hardware map
    HardwareBeep robot = new HardwareBeep();
    // Calls Library gyro to access the gyro sensor
    LibraryGyro gyro = new LibraryGyro();
    Telemetry telemetry;
    DcMotor motor;

    /**
     * The hardware class needs to be initialized before this method is called
     */
    public void init(HardwareBeep myRobot, Telemetry myTelemetry, DcMotor myMotor) {
        robot = myRobot;
        telemetry = myTelemetry;
        gyro.init(robot, telemetry);
        motor = myMotor;

    }

    /**
     * This is the method we use to call gyro drive in the Grid Navigation library
     *
     * @param speed        This allows us to input a speed when we call this method.
     * @param encoderTicks Input how far you want to drive using the encoder ticks.
     * @param angle        Input what angle you want it to drive.
     */
    public void gyroDrive(double speed,
                          int encoderTicks,
                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        telemetry.addData("In Gyro Drive method", "");
        telemetry.update();

        // In order to use the encoders you need to follow a specific pattern. The first step is to
        // stop and reset the encoders
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // The next step is to set the encoders to run
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Determine new target position, and pass to motor controller
        newLeftTarget = encoderTicks;
        newLeftTarget = encoderTicks;
        newRightTarget = encoderTicks;
        newRightTarget = encoderTicks;

        // Set Target and Turn On RUN_TO_POSITION
        robot.leftFront.setTargetPosition(newLeftTarget);
        robot.leftBack.setTargetPosition(newLeftTarget);
        robot.rightFront.setTargetPosition(newRightTarget);
        robot.rightBack.setTargetPosition(newRightTarget);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set a range clip for the speed to ensure that the robot doesn't drive faster or slower
        // than the clip
        speed = Range.clip(Math.abs(speed), 0.0, .3);
        // set all the motors to the .6 power we declared in the beginning of the program
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);

        // keep looping while motors are still active, and BOTH motors are running.
        while (robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy()
                && robot.rightBack.isBusy()) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (encoderTicks < 0)
                steer *= -1.0;

            // read the steer to determine the speed for the motors on both sides of the robot
            leftSpeed = speed + steer;
            rightSpeed = speed - steer;

            // set speed clip for the motors on both sides of the robot
            leftSpeed = Range.clip(leftSpeed, -.3, .3);
            rightSpeed = Range.clip(rightSpeed, -.3, .3);

            // setting speeds
            robot.leftFront.setPower(leftSpeed);
            robot.leftBack.setPower(leftSpeed);
            robot.rightFront.setPower(rightSpeed);
            robot.rightBack.setPower(rightSpeed);

            //telemetry
            telemetry.addData("Error", error);
            telemetry.addData("Steer", steer);
            telemetry.addData("L Speed", leftSpeed);
            telemetry.addData("R Speed", rightSpeed);
            telemetry.update();

        }
        // Stop all motion;
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    /**
     * This method calculates the error and then determines the speed and direction necessary for the robot to correct in order to drive straight
     *
     * @param speed  Input your desired speed.
     * @param angle  This is the angle the method is fed by the readings to determine the error
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        // if the error is less than or equal to the target heading don't change the motor speeds
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
            // else if the error is greater then the target heading than we get the steer by taking
            // the error and P Coefficient
        } else {
            steer = getSteer(error, PCoeff);
            // we multiply the speed by the steer and set the left speed to be the opposite of that
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFront.setPower(leftSpeed);
        robot.leftBack.setPower(leftSpeed);
        robot.rightFront.setPower(rightSpeed);
        robot.rightBack.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;


    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getAngle();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /**
     * This is the method we use to call gyro drive in the Grid Navigation library when we want to integrate the p coefficient
     *
     * @param speed        When you call the method input the desired speed
     * @param encoderTicks Input the distance you want to drive in encoder ticks
     * @param angle        What angle you would like to drive
     * @param PCoeff       Proportional Gain Coefficient
     */
    public void gyroDriveVariableP(double speed,
                                   int encoderTicks,
                                   double angle, double PCoeff) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        telemetry.addData("In Gyro Drive method", "");
        telemetry.update();

        // In order to use the encoders you need to follow a specific pattern. The first step is to
        // stop and reset the encoders
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Then we set the encoders to run
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Determine new target position, and pass to motor controller
        newLeftTarget = encoderTicks;
        newLeftTarget = encoderTicks;
        newRightTarget = encoderTicks;
        newRightTarget = encoderTicks;

        // Set Target and Turn On RUN_TO_POSITION
        robot.leftFront.setTargetPosition(newLeftTarget);
        robot.leftBack.setTargetPosition(newLeftTarget);
        robot.rightFront.setTargetPosition(newRightTarget);
        robot.rightBack.setTargetPosition(newRightTarget);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // keep looping while we are still active, and BOTH motors are running.
        do {

            telemetry.addData("libgyrodr: target angle", angle);
            telemetry.addData("libgyrodr: gyro angle", gyro.getAngle());
            // adjust relative speed based on heading error.
            error = getError(angle);

            steer = getSteer(error, PCoeff);

            // if driving in reverse, the motor correction also needs to be reversed
            if (encoderTicks < 0)
                steer *= -1.0;

            leftSpeed = speed + steer;
            rightSpeed = speed - steer;

            leftSpeed = Range.clip(leftSpeed, -.3, 1);
            rightSpeed = Range.clip(rightSpeed, -.3, 1);

            robot.leftFront.setPower(leftSpeed);
            robot.leftBack.setPower(leftSpeed);
            robot.rightFront.setPower(rightSpeed);
            robot.rightBack.setPower(rightSpeed);

            // Display drive status for the driver.
            telemetry.addData("Error", error);
            telemetry.addData("Steer", steer);
            telemetry.addData("L Speed", leftSpeed);
            telemetry.addData("R Speed", rightSpeed);
            telemetry.update();
        }
        while (robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy()
                && robot.rightBack.isBusy());

        // Stop all motion;
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


}