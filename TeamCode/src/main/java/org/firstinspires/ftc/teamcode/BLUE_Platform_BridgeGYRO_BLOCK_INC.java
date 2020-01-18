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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

@Autonomous(name = "BLUE_BLOCK", group = "Linear Opmode")
public class BLUE_Platform_BridgeGYRO_BLOCK_INC extends BaseAutoOpMode {


    double globalAngle, power = 1, correction;

    int startingSide = 1;  //Set to 1 for blue and -1 for Red

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 40.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        GetHardware();
        //GetIMU();

   /*     // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rear_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rear_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at Front %7d :%7d Rear %7d: %7d",
                front_left.getCurrentPosition(),
                front_right.getCurrentPosition(),
                rear_left.getCurrentPosition(),
                rear_right.getCurrentPosition());
        telemetry.update();





        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  48,  48, 500.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 5.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
       // encoderDrive(DRIVE_SPEED, -24, -24, 5.0);  // S3: Reverse 24 Inches with 4 Sec timeout
*/
        waitForStart();
        
        
        //Robot is strafes left then stops
        front_left.setPower(1 * startingSide);
        rear_left.setPower(-0.7 * startingSide);
        front_right.setPower(-1 * startingSide);
        rear_right.setPower(1 * startingSide);
        //sleep(250);
        sleep(725);
        Drive(DriveDirection.STOP);

        // Larson added this. This is added to have the robot move back to the wall to hopefully straighten it.
        Drive(DriveDirection.BACKWARD);
        sleep(100);

        UnfoldRobot();

        //Clamps half
        //Clamp_Left.setPosition(0.5);
        //Clamp_Right.setPosition(0.5);
        //sleep(1000);

        //Robot Drives Forward
        //Drive(DriveDirection.FORWARD);
        //sleep(225);
        //Drive(DriveDirection.STOP);

        //Clamps are down
        Clamp_Left.setPosition(1);
        Clamp_Right.setPosition(0f);
        sleep(2000);

        //Robot moves backwards
        Drive(DriveDirection.BACKWARD);
        sleep(600);


        //Robot strafes right
        front_left.setPower(-1 * startingSide);
        rear_left.setPower(.7 * startingSide);
        front_right.setPower(1 * startingSide);
        rear_right.setPower(-1 * startingSide);
        sleep(1200);
        Drive(DriveDirection.STOP);
        //sleep(600);

        //front_left.setPower(1);
        //rear_left.setPower(1);
        //front_right.setPower(-1);
        //rear_right.setPower(-1);
        //sleep(1900);

        //Turns right
        rotate(55 * startingSide, 1);

        //Robot drives platform
        Drive(DriveDirection.FORWARD);
        sleep(750);
        Drive(DriveDirection.STOP);

        //End of moving platform

        //Clamps go up
        Clamp_Left.setPosition(0f);
        Clamp_Right.setPosition(1f);
        Release_Servo.setPosition(.2);
         sleep(500);

        //Pulley system moves backwards
        top_motor.setPower(-1);
        sleep(200);

        //Control system that stops pulley
        while (Top_Sensor_Rear.getState()) {
            top_motor.setPower(0.7);
            telemetry.addData("Loop Crane ", Top_Sensor_Rear.getState());
            telemetry.update();
        }

        top_motor.setPower(0);

        //Control system that gets pulley system to go
        while (bottom_touch.getState()) {
            lift_left.setPower(1);
            lift_right.setPower(1);
            telemetry.addData("Loop Lift ", bottom_touch.getState());
            telemetry.update();
        }
        telemetry.addData("Loop Lift ", "Out Of Loop");
        telemetry.update();

        //top_motor.setPower(0);


        //  lift_left.setPower(1);
        //  lift_right.setPower(1);
        //  sleep(1500);

        //Lift
        lift_left.setPower(0);
        lift_right.setPower(0);

        //Open Claw
        Block_Pickup.setPosition(1);
        sleep(1000);


        Drive(DriveDirection.BACKWARD);
        sleep(1700);
        Drive(DriveDirection.STOP);

        //reset gyro and rotate 30
        feeder_motor.setPower(-1);
        resetAngle();
        rotate(30 * startingSide, 1);


        //turn on feeder and drive backwards
        feeder_motor.setPower(-1);
        Drive(DriveDirection.BACKWARD);
        sleep(800);
        Drive(DriveDirection.STOP);

        //keep feeder on
        feeder_motor.setPower(-1);

        //Drive Forward
        Drive(DriveDirection.FORWARD);
        sleep(800);
        Drive(DriveDirection.STOP);

        //rotate back
        resetAngle();
        rotate(-30, 1);


        //Drive Forward
        Drive(DriveDirection.FORWARD);
        sleep(1350);
        Drive(DriveDirection.STOP);

        //Close Claw
        Block_Pickup.setPosition(1);
        sleep(1000);

        //turn off feeder
        feeder_motor.setPower(0);
        //Crane Up
        lift_left.setPower(-1);
        lift_right.setPower(-1);
        sleep(1000);

        //Lift Stop
        lift_left.setPower(0);
        lift_right.setPower(0);

        //Crane Across
        top_motor.setPower(-1);
        while (Top_Sensor_Front.getState()) {
            top_motor.setPower(-1);
            telemetry.addData("Status", "Moving Crane");
            telemetry.update();
        }
        top_motor.setPower(0);

        //Lift Down
        lift_left.setPower(1);
        lift_right.setPower(1);
        sleep(750);

        //Open Claw
        Block_Pickup.setPosition(0.4);


    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = front_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
           newRightTarget = front_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            //newLeftTarget = rear_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            //newRightTarget = rear_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
          //  front_left.setTargetPosition(newLeftTarget);
            front_right.setTargetPosition(newRightTarget);
          //  rear_left.setTargetPosition(newLeftTarget);
           // rear_right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
          //  front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           front_right.setDirection(DcMotor.Direction.FORWARD);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         //   rear_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         //   rear_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
          //  front_left.setPower(Math.abs(speed));
            front_right.setPower(Math.abs(speed));
         //   rear_left.setPower(Math.abs(speed));
         //   rear_right.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            //while (opModeIsActive() &&
            //        (runtime.seconds() < timeoutS) &&
            //        (front_left.isBusy() && front_right.isBusy())) {

                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (front_right.isBusy())) {

                    // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at Front %7d :%7d, Rear %7d : %7d ",
                        front_left.getCurrentPosition(),
                        front_right.getCurrentPosition(),
                        rear_left.getCurrentPosition(),
                        rear_right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            front_left.setPower(0);
            front_right.setPower(0);
            rear_left.setPower(0);
            rear_right.setPower(0);


            // Turn off RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}

