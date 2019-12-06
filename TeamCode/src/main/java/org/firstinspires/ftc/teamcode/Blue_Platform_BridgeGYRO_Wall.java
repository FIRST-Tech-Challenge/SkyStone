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

@Autonomous(name = "Blue_Platform_BridgeGYRO_Wall", group = "Linear Opmode")
public class Blue_Platform_BridgeGYRO_Wall extends BaseAutoOpMode {


    double                  globalAngle, power = 1, correction;

    int startingSide = 1;  //Set to 1 for blue and -1 for Red


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        GetHardware();
        GetIMU();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        front_left.setPower(1 * startingSide);
        rear_left.setPower(-1 * startingSide);
        front_right.setPower(-1 * startingSide);
        rear_right.setPower(1 * startingSide);
        //sleep(250);
        sleep(500);
        Drive(DriveDirection.STOP);

        UnfoldRobot();

        //Clamp_Left.setPosition(0.5);
        //Clamp_Right.setPosition(0.5);
        //sleep(1000);

       // Drive(DriveDirection.FORWARD);
        //sleep(210);
        //Drive(DriveDirection.STOP);

        Clamp_Left.setPosition(1);
        Clamp_Right.setPosition(0f);
        sleep(1500);

        Drive(DriveDirection.BACKWARD);
        sleep(450);


        front_left.setPower(-1 * startingSide);
        rear_left.setPower(1 * startingSide);
        front_right.setPower(1 * startingSide);
        rear_right.setPower(-1 * startingSide);
        sleep(1200);

        //front_left.setPower(1);
        //rear_left.setPower(1);
        //front_right.setPower(-1);
        //rear_right.setPower(-1);
        //sleep(1900);

        rotate(60 * startingSide, 1);

        Drive(DriveDirection.FORWARD);
        sleep(600);
        Drive(DriveDirection.STOP);

        //End of moving platform

        //Clamp Up
        Clamp_Left.setPosition(0f);
        Clamp_Right.setPosition(1f);
        sleep(1000);

        //Strafe Left
        front_left.setPower(1 * startingSide);
        rear_left.setPower(-1 * startingSide);
        front_right.setPower(-1 * startingSide);
        rear_right.setPower(1 * startingSide);
        sleep(500);


        Release_Servo.setPosition(0.4);
        sleep(1000);

        top_motor.setPower(-1);
        sleep(200);


        while(Top_Sensor_Rear.getState())
        {
            top_motor.setPower(0.7);
            telemetry.addData("Loop Crane " , Top_Sensor_Rear.getState());
            telemetry.update();
        }

        top_motor.setPower(0);


        while(bottom_touch.getState())
        {
            lift_left.setPower(1);
            lift_right.setPower(1);
            telemetry.addData("Loop Lift " , bottom_touch.getState());
            telemetry.update();
        }
        telemetry.addData("Loop Lift ", "Out Of Loop");
        telemetry.update();

        //top_motor.setPower(0);


      //  lift_left.setPower(1);
      //  lift_right.setPower(1);
      //  sleep(1500);

        lift_left.setPower(0);
        lift_right.setPower(0);

        Drive(DriveDirection.BACKWARD);
        sleep(700);
        Drive(DriveDirection.STOP);












    }

}

