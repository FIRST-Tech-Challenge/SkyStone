package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class LibraryUltrasonicDrive {

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.01;     // Larger is more responsive, but also less stable
    public ElapsedTime runtime = new ElapsedTime();
    //DcMotor motor;
    HardwareBeep robot = new HardwareBeep();
    Telemetry telemetry;
    boolean readLeftSensor = false;
    int read_distance_left = 0;
    int read_distance_right = 0;

    /**
     * The hardware class needs to be initialized before this f unction is called
     */
    public void init(HardwareBeep myRobot, Telemetry myTelemetry) {
        robot = myRobot;
        telemetry = myTelemetry;
        //motor = myMotor;

    }

    public void ultrasonicDrive(double speed,
                                int encoderTicks,
                                double distance_target) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        int i = 0;

        telemetry.addData("In Ultrasonic Drive method", "");
        telemetry.update();

        telemetry.addData("Distance read", robot.leftSonic.getDistance());
        telemetry.addData("Distance read", robot.rightSonic.getDistance());
        telemetry.update();
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

//            telemetry.addData("Code pos 1", "");
//            telemetry.update();
//            sleep(2000);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);

//            telemetry.addData("Code pos 2", "");
//            telemetry.update();
//            sleep(2000);


        // keep looping while we are still active, and BOTH motors are running.
        while (robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy()) {

            if (!readLeftSensor && runtime.milliseconds() > 100) {
                telemetry.addData("Left Distance", robot.leftSonic.getDistance());
                telemetry.addData("Incrementor", i++);
                telemetry.update();
                robot.leftSonic.ping();
                readLeftSensor = true;

            }

            if (runtime.milliseconds() > 200) {

                telemetry.addData("Right Distance", robot.rightSonic.getDistance());
                telemetry.addData("Incrementor", i++);
                telemetry.update();
                robot.rightSonic.ping();
                runtime.reset();
                readLeftSensor = false;
            }


            // adjust relative speed based on heading error.
            error = getError(distance_target);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (encoderTicks < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            robot.leftFront.setPower(leftSpeed);
            robot.leftBack.setPower(leftSpeed);
            robot.rightFront.setPower(rightSpeed);
            robot.rightBack.setPower(rightSpeed);

//                telemetry.addData("Code pos 4", "");
//                telemetry.update();


            // Display drive status for the driver.
            telemetry.addData("Left Distance", read_distance_left);
            telemetry.addData("Right Distance", read_distance_right);
            telemetry.addData("Incrementor", i++);
            telemetry.addData("Error", error);
            telemetry.addData("Steer", steer);
//                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
//                telemetry.addData("Actual",  "%7d:%7d",      robot.leftFront.getCurrentPosition(),
//                telemetry.addData("Actual",  "%7d:%7d",      robot.leftBack.getCurrentPosition(),
//                                                             robot.rightFront.getCurrentPosition()));
//                                                             robot.rightBack.getCurrentPosition();
            telemetry.addData("L Speed", leftSpeed);
            telemetry.addData("R Speed", rightSpeed);
            telemetry.update();

        }

//            telemetry.addData("GyroDrive, after drive commands", "");
//            telemetry.update();
//            sleep(2000);

        //Stop all motion;
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


    boolean onHeading(double speed, double distance, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(distance);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFront.setPower(leftSpeed);
        robot.leftBack.setPower(leftSpeed);
        robot.rightFront.setPower(rightSpeed);
        robot.rightBack.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", distance);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;


    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetDistance Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetDistance) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = read_distance_left - read_distance_right - targetDistance;
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


}