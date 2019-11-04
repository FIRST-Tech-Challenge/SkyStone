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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import pkg3939.Robot3939;
import pkg3939.skystoneDetectorClass;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="REDAutoSkystoneDelivery", group="Pushbot")
//@Disabled
public class REDAutoSkystoneDelivery extends LinearOpMode {

    /* Declare OpMode members. */
    Robot3939 robot = new Robot3939();   // Use a Pushbot's hardware

    skystoneDetectorClass detector = new skystoneDetectorClass();
    int[] vals;
    private ElapsedTime     runtime = new ElapsedTime();

    private double UP = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.initMotors(hardwareMap);
        robot.initServos(hardwareMap);//servo
        robot.setFront(hardwareMap);
    //    robot.initIMU(hardwareMap);//gyro

        detector.setOffset(1.7f/8f, 1.2f/8f);
        detector.camSetup(hardwareMap);

        robot.useEncoders(false);

//        telemetry.addData("Mode", "calibrating...");
//        telemetry.update();
//        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
//        {
//            sleep(50);
//            idle();
//        }
//
//        telemetry.addData("Mode", "waiting for start");
//        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Ready to run");
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while(opModeIsActive()) {
            robot.claw.setPosition(UP);
            detector.updateVals();
            vals = detector.getVals();
            telemetry.addData("Values", vals[1] + "   " + vals[0] + "   " + vals[2]);
            telemetry.update();

            if(vals[0] == 0){//middle
                moveDistance(0.4, 1.6);//first forward
                robot.claw.setPosition(1);//grab skystone
                robot.servoLeft.setPosition(0.5);
                robot.servoRight.setPosition(0.5);
                mySleep(0.5);
                moveDistance(-0.3, 0.75);//back
                strafe(0.4, 4.2 );
                robot.claw.setPosition(UP);//let go of skystone

                robot.servoLeft.setPosition(0);//raises up
                robot.servoRight.setPosition(1);//raises up
                mySleep(0.5);
                strafe(-0.4, 1.2);//go back to park
//                moveDistance(0.4, 0.5);//second forward
//                robot.claw.setPosition(1);
//                mySleep(0.5);
//                moveDistance(-0.4, 0.45);//second back
            } else if(vals[1] == 0) {//left
                strafe(-0.4, 0.7);
                moveDistance(0.4, 1.6);//first forward
                robot.claw.setPosition(1);
                robot.servoLeft.setPosition(0.5);
                robot.servoRight.setPosition(0.5);
                mySleep(0.5);
                moveDistance(-0.3, 0.75);//back
                strafe(0.4, 4.8);
                robot.claw.setPosition(UP);//let go of skystone

                robot.servoLeft.setPosition(0);//raises up
                robot.servoRight.setPosition(1);//raises up
                mySleep(0.5);
                strafe(-0.4, 1.2);//go back to park
//                robot.claw.setPosition(UP);
//                strafe(-0.4, 1.9);
//                moveDistance(0.4, 0.5);
//                robot.claw.setPosition(1);
//                mySleep(0.5);
//                moveDistance(-0.4, 0.45);
            } else {//right
                strafe(0.4, 0.7);

                moveDistance(0.4, 1.6);//first forward
                robot.claw.setPosition(1);
                robot.servoLeft.setPosition(0.5);
                robot.servoRight.setPosition(0.5);
                mySleep(0.5);
                moveDistance(-0.3, 0.75);//back
                strafe(0.4, 3.7);//1.65
                robot.claw.setPosition(UP);//let go of skystone

                robot.servoLeft.setPosition(0);//raises up
                robot.servoRight.setPosition(1);//raises up
                mySleep(0.5);
                strafe(-0.4, 1.2);//go back to park
//                robot.claw.setPosition(UP);
//                strafe(-0.4, 1.9);
//                moveDistance(0.4, 0.5);
//                robot.claw.setPosition(1);
//                mySleep(0.5);
//                moveDistance(-0.4, 0.45);
            }
            telemetry.addData("Path", "Complete");
            telemetry.update();
            break;
        }
    }

    public void rotate(double power, double time) {
        robot.FL.setPower(power);
        robot.FR.setPower(-power);
        robot.RL.setPower(power);
        robot.RR.setPower(-power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        sleep(200);

    }

    public void strafe(double power, double time) {
        robot.FL.setPower(power);
        robot.FR.setPower(-power);
        robot.RL.setPower(-power);
        robot.RR.setPower(power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        sleep(200);
    }

    public void mySleep(double time) {//seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
        }
    }

    public void moveDistance(double power, double time) {

        robot.FL.setPower(power);
        robot.FR.setPower(power);
        robot.RL.setPower(power);
        robot.RR.setPower(power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        sleep(200);

    }


    public void moveDistanceEnc(double power, double distance) {
        robot.RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double distancePerRotation = 3.1415 * 4; //pi * diameter (inches)
        double rotations = -distance/distancePerRotation; //distance / circumference (inches)
        int encoderDrivingTarget = (int)(rotations*1120);

        if(opModeIsActive()) {
            robot.RL.setTargetPosition(encoderDrivingTarget);
            robot.RR.setTargetPosition(encoderDrivingTarget);
            robot.FL.setTargetPosition(encoderDrivingTarget);
            robot.FR.setTargetPosition(encoderDrivingTarget);

            robot.RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.RL.setPower(power);
            robot.RR.setPower(power);
            robot.FL.setPower(power);
            robot.FR.setPower(power);



            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
                //wait till motor finishes working
                telemetry.addData("Path", "Driving "+distance+" inches");
                telemetry.update();
            }

            robot.RL.setPower(0);
            robot.RR.setPower(0);
            robot.FR.setPower(0);
            robot.FL.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();

            robot.RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
