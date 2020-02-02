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

package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.*;

public class DriveTrain extends BotComponent {

    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;

    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;

    public GyroNavigator gyroNavigator = null;


    public enum InitType {
        INIT_FRONT_MOTORS,
        INIT_BACK_MOTORS,
        INIT_4WD
    }

    private boolean frontMotorsEnabled = false;
    private boolean backMotorsEnabled   = false;

    private double COUNTS_PER_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
    private double DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                             (WHEEL_DIAMETER_INCHES * 3.1415);

    private double MAX_INCHES_PER_SECOND   = 6;

    /* Constructor */
    public DriveTrain() {

    }

    public DriveTrain(Logger aLogger, OpMode aOpMode,
                String frontLeftMotorName, String frontRightMotorName,
                String backLeftMotorName, String backRightMotorName)
    {
        this(aLogger, aOpMode, frontLeftMotorName, frontRightMotorName, backLeftMotorName, backRightMotorName, null);
    }

    public DriveTrain(Logger aLogger, OpMode aOpMode,
                      String frontLeftMotorName, String frontRightMotorName,
                      String backLeftMotorName, String backRightMotorName,
                      GyroNavigator aGyroNavigator) {

        super(aLogger, aOpMode);

        gyroNavigator = aGyroNavigator;
        if (gyroNavigator == null) {
            gyroNavigator = new GyroNavigator(aLogger, aOpMode);
        }

        frontLeftMotor = initMotor(frontLeftMotorName, DcMotor.Direction.REVERSE);
        frontRightMotor = initMotor(frontRightMotorName);

        backLeftMotor = initMotor(backLeftMotorName, DcMotor.Direction.REVERSE);
        backRightMotor = initMotor(backRightMotorName);

    }

    public void init() {
        init(InitType.INIT_4WD);
    }

    public void init(InitType initType) {

        switch (initType) {

            case INIT_FRONT_MOTORS:
                frontMotorsEnabled = (frontLeftMotor != null) && (frontRightMotor != null);
                isAvailable = frontMotorsEnabled;
                break;

            case INIT_BACK_MOTORS:
                backMotorsEnabled = (backLeftMotor != null) && (backRightMotor != null);
                isAvailable = backMotorsEnabled;
                break;

            case INIT_4WD:
                frontMotorsEnabled = (frontLeftMotor != null) && (frontRightMotor != null);
                backMotorsEnabled = (backLeftMotor != null) && (backRightMotor != null);
                isAvailable = frontMotorsEnabled && backMotorsEnabled;
                break;

        }

        logger.logInfo("DriveTrain","isAvailable: %b", isAvailable);

    }

    public void setLeftMotorsPower(double power){
        if (frontMotorsEnabled) {
            frontLeftMotor.setPower(power);
        }
        if (backMotorsEnabled) {
            backLeftMotor.setPower(power);
        }
    }

    public void setRightMotorsPower(double power){
        if (frontMotorsEnabled) {
            frontRightMotor.setPower(power);
        }
        if (backMotorsEnabled) {
            backRightMotor.setPower(power);
        }
    }


    public void move(double seconds, double leftPower, double rightPower) {

        ElapsedTime runtime = new ElapsedTime();

        setLeftMotorsPower(leftPower);
        setRightMotorsPower(rightPower);

        runtime.reset();
        while(runtime.seconds() < seconds && opModeIsActive()) {
            opMode.telemetry.addData("Path", "Time: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        stop();

    }

    public void moveForward(double seconds, double power)
    {
        move(seconds,power, power);
    }

    public void moveBackward(double seconds, double power)
    {
        // move forwards with negative power
        move(seconds, -power, -power);
    }

    public void updateMotorsTankDrive(double leftY, double rightY) {

        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -leftY;
        right = -rightY;

        setLeftMotorsPower(left);
        setRightMotorsPower(right);

    }

    public void stop() {
        setLeftMotorsPower(0.0);
        setRightMotorsPower(0.0);
    }

    public void crabLeft(double seconds) {

        logger.logDebug("crabLeft", "seconds: %f", seconds);
        ElapsedTime runtime = new ElapsedTime();

        // leftX -1, rightY -1
        updateMotorsMechanumDrive(-1, 0, 0, -1);

        runtime.reset();
        while(runtime.seconds() < seconds && opModeIsActive()) {
            opMode.telemetry.addData("Path", "Time: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        stop();
    }

    public void crabRight(double seconds) {

        logger.logDebug("crabRight", "seconds: %f", seconds);
        ElapsedTime runtime = new ElapsedTime();

        // crabRight = lefty -1, rightX 1
        updateMotorsMechanumDrive(1, 0, 0, -1);

        runtime.reset();
        while(runtime.seconds() < seconds && opModeIsActive()) {
            opMode.telemetry.addData("Path", "Time: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        stop();
    }

    public void updateMotorsMechanumDrive(double leftX, double leftY, double rightX, double rightY) {


        double lX = -leftX;
        double lY = leftY;
        double rX = -rightX;
        double rY = rightY;

        double r = Math.hypot(lX, lY);
        double robotAngle = Math.atan2(lY, lX) - Math.PI / 4;

        final double v1 = r * Math.cos(robotAngle) + rX;
        final double v2 = r * Math.sin(robotAngle) - rX;
        final double v3 = r * Math.sin(robotAngle) + rX;
        final double v4 = r * Math.cos(robotAngle) - rX;

        if (frontMotorsEnabled) {
            frontLeftMotor.setPower(v1);
            frontRightMotor.setPower(v2);

        }
        if (backMotorsEnabled) {
            backLeftMotor.setPower(v3);
            backRightMotor.setPower(v4);
        }

        if (!opModeIsActive()) { stop(); }


    }


    public void disableEncoders() {

        if (frontMotorsEnabled) {
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (backMotorsEnabled) {
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    public void resetEncoders() {

        if (frontMotorsEnabled) {
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (backMotorsEnabled) {
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        logger.logDebug("resetEncoders",  "Front:  Left:%7d Right:%7d", getFrontLeftPosition(), getFrontRightPosition());
        logger.logDebug("resetEncoders",  "Back:   Left:%7d Right:%7d", getBackLeftPosition(), getBackRightPosition());

    }

    private void setTargetPositions(int leftTarget, int rightTarget) {

        if (frontMotorsEnabled) {
            frontLeftMotor.setTargetPosition(leftTarget);
            frontRightMotor.setTargetPosition(rightTarget);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (backMotorsEnabled) {
            backLeftMotor.setTargetPosition(leftTarget);
            backRightMotor.setTargetPosition(rightTarget);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }

    private boolean encodersAreBusy() {
        if (backMotorsEnabled) {
            return (backLeftMotor.isBusy() && backRightMotor.isBusy());
        } else {
            return (frontLeftMotor.isBusy() && frontRightMotor.isBusy());
        }
    }

    public void gyroEncoderDrive(double power,
                             double leftInches, double rightInches,
                             double timeoutSeconds) {

        boolean useGyro = true;
        encoderDrive(power,leftInches, rightInches, timeoutSeconds, useGyro);
        
    }

    public void crabEncoderLeft(double power, double inches) {
        double timeoutSeconds = (1 / Math.abs(power)) * MAX_INCHES_PER_SECOND;

        int newTarget;

        resetEncoders();

        ElapsedTime runtime = new ElapsedTime();

        if (opModeIsActive()) {

            if (frontMotorsEnabled) {
                newTarget = frontRightMotor.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                frontRightMotor.setTargetPosition(newTarget);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                newTarget = backRightMotor.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                backRightMotor.setTargetPosition(newTarget);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // reset the timeout time and start motion.
            runtime.reset();
            updateMotorsMechanumDrive(-power, 0, 0, -power);

            logger.setDebugFilter("crabEncoderLeft");

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutSeconds) && frontRightMotor.isBusy()) {

                logger.logDebug("crabEncoderLeft", "Inches: %f Target: %7d", inches, newTarget);
                logger.logDebug("crabEncoderLeft", "Front:  Left:%7d Right:%7d", getFrontLeftPosition(), getFrontRightPosition());
                logger.logDebug("crabEncoderLeft", "Back:   Left:%7d Right:%7d", getBackLeftPosition(), getBackRightPosition());
                logger.logDebug("crabEncoderLeft", "runtime.seconds: %f timeout: %f", runtime.seconds(), timeoutSeconds);

                logger.incrementDebugFilterCount();
            }

            // Stop all motion;
            stop();

            logger.clearDebugFilter();
            logger.logDebug("crabEncoderLeft", "Inches: %f Target: %7d", inches, newTarget);
            logger.logDebug("crabEncoderLeft", "Front:  Left:%7d Right:%7d", getFrontLeftPosition(), getFrontRightPosition());
            logger.logDebug("crabEncoderLeft", "Back:   Left:%7d Right:%7d", getBackLeftPosition(), getBackRightPosition());
            logger.logDebug("crabEncoderLeft", "runtime.seconds: %f timeout: %f", runtime.seconds(), timeoutSeconds);

            disableEncoders();

        }


    }

    public void crabEncoderRight(double power, double inches) {
        double timeoutSeconds = (1 / Math.abs(power)) * MAX_INCHES_PER_SECOND;

        int newTarget;

        resetEncoders();

        ElapsedTime runtime = new ElapsedTime();

        if (opModeIsActive()) {

            if (frontMotorsEnabled) {
                newTarget = backLeftMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                backLeftMotor.setTargetPosition(newTarget);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                newTarget = backLeftMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                backLeftMotor.setTargetPosition(newTarget);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // reset the timeout time and start motion.
            runtime.reset();
            updateMotorsMechanumDrive(power, 0, 0, -power);


            logger.setDebugFilter("crabEncoderLeft");

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutSeconds) && backLeftMotor.isBusy()) {

                logger.logDebug("crabEncoderLeft", "Inches: %f Target: %7d", inches, newTarget);
                logger.logDebug("crabEncoderLeft", "Front:  Left:%7d Right:%7d", getFrontLeftPosition(), getFrontRightPosition());
                logger.logDebug("crabEncoderLeft", "Back:   Left:%7d Right:%7d", getBackLeftPosition(), getBackRightPosition());
                logger.logDebug("crabEncoderLeft", "runtime.seconds: %f timeout: %f", runtime.seconds(), timeoutSeconds);

                logger.incrementDebugFilterCount();
            }

            // Stop all motion;
            stop();

            logger.clearDebugFilter();
            logger.logDebug("crabEncoderLeft", "Inches: %f Target: %7d", inches, newTarget);
            logger.logDebug("crabEncoderLeft", "Front:  Left:%7d Right:%7d", getFrontLeftPosition(), getFrontRightPosition());
            logger.logDebug("crabEncoderLeft", "Back:   Left:%7d Right:%7d", getBackLeftPosition(), getBackRightPosition());
            logger.logDebug("crabEncoderLeft", "runtime.seconds: %f timeout: %f", runtime.seconds(), timeoutSeconds);


            disableEncoders();

        }


    }

    public void encoderDrive(double power,
                             double leftInches, double rightInches,
                             double timeoutSeconds) {
        boolean useGyro = false;
        encoderDrive(power,leftInches, rightInches, timeoutSeconds, useGyro);

    }

    public void encoderDrive(double power, double inches) {
        encoderDrive(power, inches, inches);
    }

    public void encoderDrive(double power, double leftInches, double rightInches) {
        double timeoutSeconds = (1 / Math.abs(power)) * MAX_INCHES_PER_SECOND;
        encoderDrive(power, leftInches, rightInches, timeoutSeconds, false);
    }

    public void encoderDrive(double power,
                             double leftInches, double rightInches,
                             double timeoutSeconds,
                             boolean useGyro) {

        int targetAngle =0;
        if (useGyro && gyroNavigator.isAvailable) {
            // keep the current angle as the target to stay on
            targetAngle = (int) gyroNavigator.getAngle();
        }

        int newLeftTarget;
        int newRightTarget;

        resetEncoders();

        ElapsedTime runtime = new ElapsedTime();

        // Ensure that the opmode is still active - ToDo: Need to pull this from the opmode
        // boolean opModeIsActive = true;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            if (backMotorsEnabled) {
                newLeftTarget = backLeftMotor.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget = backRightMotor.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
            } else {
                newLeftTarget = frontLeftMotor.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget = frontRightMotor.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
            }

            setTargetPositions(newLeftTarget, newRightTarget);

            // reset the timeout time and start motion.
            runtime.reset();

            setLeftMotorsPower(power);
            setRightMotorsPower(power);
            logger.logDebug("driveByEncoder", "===== [ Start ]");

            logger.setDebugFilter("encoderDrive");

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutSeconds) && encodersAreBusy()) {

                logger.logDebug("encoderDrive", "Inches: Left:%f Right:%f", leftInches, rightInches);
                logger.logDebug("encoderDrive", "Target: Left:%7d Right:%7d", newLeftTarget, newRightTarget);
                logger.logDebug("encoderDrive", "Front:  Left:%7d Right:%7d", getFrontLeftPosition(), getFrontRightPosition());
                logger.logDebug("encoderDrive", "Back:   Left:%7d Right:%7d", getBackLeftPosition(), getBackRightPosition());
                logger.logDebug("encoderDrive", "runtime.seconds: %f timeout: %f", runtime.seconds(), timeoutSeconds);

                logger.incrementDebugFilterCount();

                if (useGyro && gyroNavigator.isAvailable) {
                    if (Math.abs(gyroNavigator.getAngle() - targetAngle) > 2) {
                        gyroRotate(targetAngle, power);
                    }
                }

                opMode.telemetry.update();
            }

            logger.clearDebugFilter();

            // Stop all motion;
            stop();
            logger.logDebug("encoderDrive", "===== [ Stop ]");

            logger.logDebug("encoderDrive", "Inches: Left:%f Right:%f", leftInches, rightInches);
            logger.logDebug("encoderDrive", "Target: Left:%7d Right:%7d", newLeftTarget, newRightTarget);
            logger.logDebug("encoderDrive", "Front:  Left:%7d Right:%7d", getFrontLeftPosition(), getFrontRightPosition());
            logger.logDebug("encoderDrive", "Back:   Left:%7d Right:%7d", getBackLeftPosition(), getBackRightPosition());
            logger.logDebug("encoderDrive", "runtime.seconds: %f timeout: %f", runtime.seconds(), timeoutSeconds);

            disableEncoders();

        }


    }

    private int getFrontLeftPosition() {
        return frontLeftMotor.getCurrentPosition();        
    }

    private int getFrontRightPosition() {
        return frontRightMotor.getCurrentPosition();
    }

    private int getBackLeftPosition() {
        return backLeftMotor.getCurrentPosition();
    }

    private int getBackRightPosition() {
        return backRightMotor.getCurrentPosition();
    }


    public void gyroRotate(double degrees, double power) {
        boolean isRelative = true;
        gyroRotate(degrees, power, isRelative);
    }

    public void gyroRotate(double degrees, double power, boolean isRelative) {
        boolean adjustAngle = true;
        gyroRotate(degrees, power, isRelative, adjustAngle);
    }

    public void gyroRotate(double degrees, double power, boolean isRelative, boolean adjustAngle) {

        if (!gyroNavigator.isAvailable) {
            logger.logErr("gyroRotate", "Error: %s","gyroNavigator is not available");
            return;
        }

        double currentAngle = gyroNavigator.getAngle();
        double targetAngle = degrees;
        double adjustedPower = power;

        if (isRelative) { targetAngle = currentAngle + degrees;}

        boolean rotationComplete = false;
        logger.setDebugFilter("gyroRotate");

        while (opModeIsActive() && !rotationComplete) {

            if (Math.abs((int)currentAngle - (int)targetAngle) < 5 ) {
                adjustedPower = 0.25;
            }

            logger.logDebug("gyroRotate", "degrees: %f, power: %f, adjustedPower: %f", degrees, power, adjustedPower);
            double leftPower = 0;
            double rightPower = 0;

            if (targetAngle < currentAngle)
            {   // turn left.
                logger.logDebug("gyroRotate", "turning left");
                leftPower = adjustedPower;
                rightPower = - adjustedPower;
/*
                int compareResult = Double.compare(currentAngle, targetAngle);
                if (compareResult >= 0) {
                    rotationComplete = true;
                }
*/
                if (currentAngle <= targetAngle) {
                    rotationComplete = true;
                }

            } else if (targetAngle > currentAngle) {   // turn right.
                logger.logDebug("gyroRotate", "turning right");
                leftPower = - adjustedPower;
                rightPower = adjustedPower;
                if (currentAngle >= targetAngle) {
                    rotationComplete = true;
                }
            } else {
                rotationComplete = true;
            }

            if (rotationComplete) { logger.clearDebugFilter(); };
            logger.logDebug("gyroRotate", "currentAngle: %f, targetAngle: %f", currentAngle, targetAngle);
            logger.logDebug("gyroRotate", "rotationComplete: %b", rotationComplete);
            logger.logDebug("gyroRotate", "adjustAngle: %b", adjustAngle);
            logger.incrementDebugFilterCount();

            if (!rotationComplete) {
                setLeftMotorsPower(leftPower);
                setRightMotorsPower(rightPower);
            } else {
                stop();
            }

            opMode.telemetry.update();
            idle();

            currentAngle = gyroNavigator.getAngle();

        }

        if (adjustAngle) {
            if (opModeIsActive() && currentAngle != targetAngle && power > 0.25) {
                logger.logDebug("gyroRotate", "ADJUST ANGLE");
                gyroRotate(targetAngle, power / 2, false);
            }
        }
    }


    public void encoderDrive2(double Lspeed, double Rspeed, double Inches, double timeoutS, double rampup) throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();

        //initialise some variables for the subroutine
        int newLeftTarget;
        int newRightTarget;

        // Determine new target position, and pass to motor controller
        if (backMotorsEnabled && frontMotorsEnabled) {
            newLeftTarget = ( backLeftMotor.getCurrentPosition() + frontLeftMotor.getCurrentPosition() ) /2 - (int) (Inches * COUNTS_PER_INCH);
            newRightTarget = ( backRightMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition() ) /2 - (int) (Inches * COUNTS_PER_INCH);
        } else if (backMotorsEnabled) {
            newLeftTarget = backLeftMotor.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);
            newRightTarget = backRightMotor.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);
        } else {
            newLeftTarget = frontLeftMotor.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);
            newRightTarget = frontRightMotor.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);
        }

        // reset the timeout time and start motion.
        runtime.reset();
        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while ( (runtime.seconds() < timeoutS) &&
                (Math.abs(backLeftMotor.getCurrentPosition() + frontLeftMotor.getCurrentPosition()) /2 < newLeftTarget  &&
                        Math.abs(backRightMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition())/2 < newRightTarget))
        {
            double rem = (Math.abs(backLeftMotor.getCurrentPosition())
                       + Math.abs(frontLeftMotor.getCurrentPosition())
                       + Math.abs(backRightMotor.getCurrentPosition())
                       + Math.abs(frontRightMotor.getCurrentPosition()))/4;

            double NLspeed;
            double NRspeed;
            //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over
            //the amount of time you set for this SubRun
            double R = runtime.seconds();
            if (R < rampup) {
                double ramp = R / rampup;
                NLspeed = Lspeed * ramp;
                NRspeed = Rspeed * ramp;
            }

            //Keep running until you are about two rotations out
            else if(rem > (1000) )
            {
                NLspeed = Lspeed;
                NRspeed = Rspeed;
            }
            //start slowing down as you get close to the target
            else if(rem > (200) && (Lspeed*.2) > .1 && (Rspeed*.2) > .1) {
                NLspeed = Lspeed * (rem / 1000);
                NRspeed = Rspeed * (rem / 1000);
            }
            //minimum speed
            else {
                NLspeed = Lspeed * .2;
                NRspeed = Rspeed * .2;

            }
            //Pass the seed values to the motors
            setRightMotorsPower(NRspeed);
            setLeftMotorsPower(NLspeed);
        }

        // Stop all motion;
        //Note: This is outside our while statement, this will only activate once the time, or distance has been met
        stop();

        // show the driver how close they got to the last target
        logger.logDebug("encoderDrive2", "Target: Left:%7d Right:%7d", newLeftTarget,  newRightTarget);
        logger.logDebug("encoderDrive2", "Front:  Left:%7d Right:%7d", getFrontLeftPosition(), getFrontRightPosition());
        logger.logDebug("encoderDrive2", "Back:   Left:%7d Right:%7d", getBackLeftPosition(), getBackRightPosition());
        logger.logDebug("encoderDrive2", "runtime.seconds: %f", runtime.seconds());


        //setting resetC as a way to check the current encoder values easily
        double resetC = (Math.abs(backLeftMotor.getCurrentPosition())
                + Math.abs(frontLeftMotor.getCurrentPosition())
                + Math.abs(backRightMotor.getCurrentPosition())
                + Math.abs(frontRightMotor.getCurrentPosition()));
        //Get the motor encoder resets in motion

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //keep waiting while the reset is running
        while (Math.abs(resetC) > 0){
            resetC = (Math.abs(backLeftMotor.getCurrentPosition())
                    + Math.abs(frontLeftMotor.getCurrentPosition())
                    + Math.abs(backRightMotor.getCurrentPosition())
                    + Math.abs(frontRightMotor.getCurrentPosition()));
            idle();
        }

        // switch the motors back to RUN_USING_ENCODER mode
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //give the encoders a chance to switch modes.
        pause(.25);
    }


}

