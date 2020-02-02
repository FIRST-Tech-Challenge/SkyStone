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

import org.firstinspires.ftc.teamcode.components.BotComponent;
import org.firstinspires.ftc.teamcode.components.GyroNavigator;
import org.firstinspires.ftc.teamcode.components.Logger;

import java.util.ArrayList;
import java.util.List;

public class DriveTrainSimple extends BotComponent {

    private String motorFLName = "frontLeftMotor";
    private String motorFRName = "frontRightMotor";
    private String motorBLName = "backLeftMotor";
    private String motorBRName = "backRightMotor";


    public DcMotor motorFL = null;
    public DcMotor motorFR = null;
    public DcMotor motorBL = null;
    public DcMotor motorBR = null;

    List<DcMotor> motorList = new ArrayList<DcMotor>();


    private double COUNTS_PER_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
    private double DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private double MAX_INCHES_PER_SECOND   = 6;

    /* Constructor */
    public DriveTrainSimple() {

    }

    public DriveTrainSimple(Logger aLogger, OpMode aOpMode,
                            String aMotorFLName, String aMotorFRName,
                            String aMotorBLName, String aMotorBRName)
    {
        super(aLogger, aOpMode);
        motorFLName = aMotorFLName;
        motorFRName = aMotorFRName;
        motorBLName = aMotorBLName;
        motorBRName = aMotorBRName;
    }

    public void init() {

        motorFL = initMotor(motorFLName, DcMotor.Direction.REVERSE);
        motorFR = initMotor(motorFRName);

        motorBL = initMotor(motorBLName, DcMotor.Direction.REVERSE);
        motorBR = initMotor(motorBRName);

        motorList.add(motorFL);
        motorList.add(motorFR);
        motorList.add(motorBL);
        motorList.add(motorBR);

        isAvailable = motorFL != null && motorFR != null && motorBL != null && motorBR != null;

        logger.logInfo("DriveTrainSimple","isAvailable: %b", isAvailable);

    }

    private void setPower(double speedFactor, double powerFL, double powerFR, double powerBL, double powerBR) {
        motorFL.setPower(speedFactor * powerFL);
        motorFR.setPower(speedFactor * powerFR);
        motorBL.setPower(speedFactor * powerBL);
        motorBL.setPower(speedFactor * powerBR);
    }

    private void driveByTime(double power, double seconds) {
        driveByTime(power, seconds, 0.10, 0.05);
    }

    private void driveByTime(double power, double seconds, double startSpeed, double increment) {

        double speedFactor = startSpeed;

        setPower(speedFactor, power, power, power, power);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while(opModeIsActive() && runtime.seconds() < seconds) {
            if  (speedFactor < 1) {
                speedFactor += increment;
                setPower(speedFactor, power, power, power, power);
            }
            opMode.telemetry.addData("Path", "Time: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        stop();

    }

    public void stop() {
        setPower(1, 0, 0, 0, 0);
    }

    public void driveMechanum(double leftX, double leftY, double rightX, double rightY) {


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

        motorFL.setPower(v1);
        motorFR.setPower(v2);
        motorBL.setPower(v3);
        motorBR.setPower(v4);

        if (!opModeIsActive()) { stop(); }


    }

    private static final int DIRECTION_LEFT = -1;
    private static final int DIRECTION_RIGHT = 1;
    private static final int DIRECTION_STRAIGHT = 0;

    public void driveByEncoder(double power, double inches) {
        logger.logDebug("driveByEncoder", "===== [ Drive Straight ] %f power, %f inches", power, inches);
        driveByEncoder(power, inches, DIRECTION_STRAIGHT);
    }

    public void crabByEncoderLeft(double power, double inches) {
        logger.logDebug("driveByEncoder", "===== [ Crab Left ] %f power, %f inches", power, inches);
        driveByEncoder(power, inches, DIRECTION_LEFT);
    }

    public void crabByEncoderRight(double power, double inches) {
        logger.logDebug("driveByEncoder", "===== [ Crab Right ] %f power, %f inches", power, inches);
        driveByEncoder(power, inches, DIRECTION_RIGHT);
    }
    private void driveByEncoder(double power, double inches, int direction) {

        resetEncoders(motorFL, motorFR, motorBL, motorBR);

        int targetFL = 0 - (int)Math.round(inches * COUNTS_PER_INCH);
        int targetFR = 0 - (int)Math.round(inches * COUNTS_PER_INCH);
        int targetBL = 0 - (int)Math.round(inches * COUNTS_PER_INCH);
        int targetBR = 0 - (int)Math.round(inches * COUNTS_PER_INCH);

        if (direction != 0) {
            targetFL = direction * targetFL;
            targetFR = (0 - direction) * targetFR;
            targetBL = (0 - direction) * targetBL;
            targetBR = direction * targetBR;
        }

        setEncoderTargets(targetFL, targetFR, targetBL, targetBR);

        double timeoutSeconds = (1 / Math.abs(power)) * MAX_INCHES_PER_SECOND;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        setPower(1, power, power, power, power);

        logger.setDebugFilter("driveByEncoder");

        logger.logDebug("driveByEncoder", "===== [ Start ]");
        logger.logDebug("driveByEncoder", "Inches: %f     Direction: %7d", inches, direction);
        logger.logDebug("driveByEncoder", "Front Targets: Left:%7d Right:%7d", targetFL, targetFR);
        logger.logDebug("driveByEncoder", "Back  Target:  Left:%7d Right:%7d", targetBL, targetBR);

        logger.logDebug("driveByEncoder", "===== [ Position ]");
        logger.logDebug("driveByEncoder", "Front:  Left:%7d Right:%7d", motorFL.getCurrentPosition(), motorFR.getCurrentPosition());
        logger.logDebug("driveByEncoder", "Back:   Left:%7d Right:%7d", motorBL.getCurrentPosition(), motorBR.getCurrentPosition());
        logger.logDebug("driveByEncoder", "runtime.seconds: %f timeout: %f", runtime.seconds(), timeoutSeconds);


        while ( opModeIsActive()
                && runtime.seconds() < timeoutSeconds
                //&& motorsAreBusy(motorFL, motorBL)
                && motorsAreBusy(motorFL, motorFR, motorBL, motorBR)
                )
        {

            logger.logDebug("driveByEncoder", "Target: Left:%7d Right:%7d", motorFL.getTargetPosition(), motorFR.getTargetPosition());
            logger.logDebug("driveByEncoder", "Front:  Left:%7d Right:%7d", motorFL.getCurrentPosition(), motorFR.getCurrentPosition());
            logger.logDebug("driveByEncoder", "Target: Left:%7d Right:%7d", motorBL.getTargetPosition(), motorBR.getTargetPosition());
            logger.logDebug("driveByEncoder", "Back:   Left:%7d Right:%7d", motorBL.getCurrentPosition(), motorBR.getCurrentPosition());
            logger.logDebug("driveByEncoder", "runtime.seconds: %f timeout: %f", runtime.seconds(), timeoutSeconds);

            logger.incrementDebugFilterCount();
            opMode.telemetry.update();
        }

        logger.clearDebugFilter();

        // Stop all motion;
        stop();
        logger.logDebug("driveByEncoder", "===== [ Stop ]");

        logger.logDebug("driveByEncoder", "Inches: %f", inches);
        logger.logDebug("driveByEncoder", "Target: Left:%7d Right:%7d", motorFL.getTargetPosition(), motorFR.getTargetPosition());
        logger.logDebug("driveByEncoder", "Front:  Left:%7d Right:%7d", motorFL.getCurrentPosition(), motorFR.getCurrentPosition());
        logger.logDebug("driveByEncoder", "Target: Left:%7d Right:%7d", motorBL.getTargetPosition(), motorBR.getTargetPosition());
        logger.logDebug("driveByEncoder", "Back:   Left:%7d Right:%7d", motorBL.getCurrentPosition(), motorBR.getCurrentPosition());
        logger.logDebug("driveByEncoder", "runtime.seconds: %f timeout: %f", runtime.seconds(), timeoutSeconds);

        // disable encoders;
        for (DcMotor m : motorList) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    private void resetEncoders(DcMotor...ms) {
        for(DcMotor m : ms) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private void setEncoderTargets(int targetFL, int targetFR, int targetBL, int targetBR) {
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);
        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
    }

    private static final int MOTORS_BUSY_THRESHOLD = 20;

    private boolean motorsAreBusy(DcMotor...ms) {
        int total = 0;
        for(DcMotor m : ms) {
            if (m.isBusy()) {
                final int c = Math.abs(m.getCurrentPosition());
                final int t = Math.abs(m.getTargetPosition());
                total = total + Math.max(0, t-c);
            }
        }

        //logger.logDebug("motorsAreBusy", "total: %7d ms.length: %7d", total, ms.length);

        total = total / ms.length;
        return total > MOTORS_BUSY_THRESHOLD;

    }


}

