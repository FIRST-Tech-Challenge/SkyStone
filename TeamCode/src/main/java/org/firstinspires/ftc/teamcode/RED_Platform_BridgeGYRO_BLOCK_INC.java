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


/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

@Autonomous(name = "RED_BLOCK", group = "Linear Opmode")
public class RED_Platform_BridgeGYRO_BLOCK_INC extends BaseAutoOpMode {


    double globalAngle, power = 1, correction;

    int startingSide = -1;  //Set to 1 for blue and -1 for Red


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Assigns hardware devices names and values

        GetHardware();
        //GetIMU();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Robot is strafes right then stops
        front_left.setPower(1 * startingSide);
        rear_left.setPower(-.5 * startingSide);
        front_right.setPower(-.5 * startingSide);
        rear_right.setPower(.5 * startingSide);
        //sleep(250);
        sleep(725);
        Drive(DriveDirection.STOP);

        // Larson added this. This is added to have the robot move back to the wall to hopefully straighten it.
        Drive(DriveDirection.BACKWARD);
        sleep(100);

        //Unfolds the Robot
        //DON'T EDIT UnfoldRobot() WITHOUT PERMISSION FROM ALL THE CODERS

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
        sleep(1500);

        //Robot moves backwards
        Drive(DriveDirection.BACKWARD);
        sleep(850);


        //Robot strafes left
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
        sleep(700);
        Drive(DriveDirection.STOP);

        //End of moving platform

        //Clamps go up
        Clamp_Left.setPosition(0f);
        Clamp_Right.setPosition(1f);
        Release_Servo.setPosition(0.4);
        // sleep(500);

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
        Block_Pickup.setPosition(0.4);
        sleep(1000);

        //Robot drives backwards
        Drive(DriveDirection.BACKWARD);
        sleep(1800);
        Drive(DriveDirection.STOP);

        //reset gyro and rotate 30
        feeder_motor.setPower(-1);
        resetAngle();
        rotate(-28, 1);


        //turn on feeder and drive backwards
        feeder_motor.setPower(-1);
        Drive(DriveDirection.BACKWARD);
        sleep(850);
        Drive(DriveDirection.STOP);

        //keep feeder on
        feeder_motor.setPower(-1);

        //Drive Forward
        Drive(DriveDirection.FORWARD);
        sleep(875);
        Drive(DriveDirection.STOP);

        //rotate back
        resetAngle();
        rotate(40, 1);


        //Drive Forward
        Drive(DriveDirection.FORWARD);
        sleep(1800);
        Drive(DriveDirection.STOP);

        //Close Claw
        Block_Pickup.setPosition(1);
        sleep(1000);

        //turn off feeder
        feeder_motor.setPower(0);
        //Crane Up
        lift_left.setPower(-1);
        lift_right.setPower(-1);
        sleep(800);

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

        //Open Claw
        Block_Pickup.setPosition(0.4);
        sleep(1000);

        //Crane Across
        top_motor.setPower(-1);
        while (Top_Sensor_Rear.getState()) {
            top_motor.setPower(1);
            telemetry.addData("Status", "Moving Crane");
            telemetry.update();
        }
        top_motor.setPower(0);

        //Lift Down
        lift_left.setPower(1);
        lift_right.setPower(1);
        sleep(900);

        //Robot drives backwards
        Drive(DriveDirection.BACKWARD);
        sleep(650);


    }



}

