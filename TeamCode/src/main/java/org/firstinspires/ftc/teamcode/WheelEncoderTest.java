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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="autodrivebasic", group = "Maccabot")

public class WheelEncoderTest extends LinearOpMode {

    /* Declare OpMode members. */

    private DcMotor front_left, front_right, back_left, back_right, intake_left, intake_right;
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 250 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 3.6 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.1;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        front_left = hardwareMap.dcMotor.get("front_left"); // Port 0
        front_right = hardwareMap.dcMotor.get("front_right"); // Port 1
        back_left = hardwareMap.dcMotor.get("back_left"); // Port 2
        back_right = hardwareMap.dcMotor.get("back_right"); // Port 3
        intake_left = hardwareMap.dcMotor.get("intake_left");
        intake_right = hardwareMap.dcMotor.get("intake_right");

        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);


        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                front_right.getCurrentPosition(),
                front_left.getCurrentPosition(),
                back_left.getCurrentPosition(),
                back_right.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  -2,  2, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
       // encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout


        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget2;
        int newLeftTarget;
        int newRightTarget2;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newLeftTarget2 = front_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftTarget = back_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget2 = front_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightTarget = back_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            back_left.setTargetPosition(newLeftTarget);
            back_right.setTargetPosition(newRightTarget);
            front_right.setTargetPosition(newRightTarget2);
            front_left.setTargetPosition(newLeftTarget2);

            // Turn On RUN_TO_POSITION
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            back_right.setPower(Math.abs(speed));
            back_left.setPower(Math.abs(speed));
            front_left.setPower(Math.abs(speed));
            front_right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (back_left.isBusy() && back_right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget, newLeftTarget2, newRightTarget2);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        front_right.getCurrentPosition(),
                        front_left.getCurrentPosition(),
                        back_left.getCurrentPosition(),
                        back_right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            back_left.setPower(0);
            back_right.setPower(0);
            front_right.setPower(0);
            front_left.setPower(0);


            // Turn off RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}
/*
JACOB PEARL I SWEAR TO FUCKING CHRIST IF YOU DELETE THIS I WILL MURDER YOUR WHOLE FAMILY!

@Autonomous(name="autodrivebasic")

public class WheelEncoderTest extends LinearOpMode {

       private DcMotor front_left, front_right, back_left, back_right, intake_left, intake_right;

        @Override
        public void runOpMode() throws InterruptedException {

            // opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");

           front_left = hardwareMap.dcMotor.get("front_left"); // Port 0
        front_right = hardwareMap.dcMotor.get("front_right"); // Port 1
        back_left = hardwareMap.dcMotor.get("back_left"); // Port 2
        back_right = hardwareMap.dcMotor.get("back_right"); // Port 3
        intake_left = hardwareMap.dcMotor.get("intake_left");
        intake_right = hardwareMap.dcMotor.get("intake_right");

            back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);


        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            motorFrontLeft.setTargetPosition(2440);
            motorFrontRight.setTargetPosition(2440);
            motorBackLeft.setTargetPosition(2440);
            motorBackRight.setTargetPosition(2440);

            leftFrontPos = motorLeftFront.getCurrentPosition();
            leftBackPos = motorLeftBack.getCurrentPosition();
            rightFrontPos = motorFrontRight.getCurrentPosition();
            rightBackPos = motorBackRight.getCurrentPosition();

            motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

            while(leftBackPos < 2440 && leftFrontPos < 2440 && rightBackPos < 2440 && rightFrontPos < 2440){
                motorFrontLeft.setPower(-0.5);
                motorFrontRight.setPower(-0.5);
                motorBackLeft.setPower(-0.5);
                motorBackRight.setPower(-0.5);

                leftFrontPos = motorLeftFront.getCurrentPosition();
                leftBackPos = motorLeftBack.getCurrentPosition();
                rightFrontPos = motorFrontRight.getCurrentPosition();
                rightBackPos = motorBackRight.getCurrentPosition();
            }

            telemetry.addData("2 ", "motorFrontLeft:  " + String.format("%d", motorFrontLeft.getTargetPosition()));
            telemetry.addData("3 ", "motorFrontRight:  " + String.format("%d", motorFrontRight.getTargetPosition()));
            telemetry.addData("4 ", "motorBackLeft:  " + String.format("%d", motorBackLeft.getTargetPosition()));
            telemetry.addData("5 ", "motorBackRight:  " + String.format("%d", motorBackRight.getTargetPosition()));
        }
    }
 */