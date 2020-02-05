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

import android.database.sqlite.SQLiteException;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

@Autonomous(name = "NEW AUTO TEST SPIT OUT1", group = "Linear Opmode")
public class NEW_AUTO_TEST_SPIT_OUT1 extends BaseAutoOpModeV2 {



    double globalAngle, power = 1, correction;

    int startingSide = -1;  //Set to 1 for blue and -1 for Red
    boolean problemChild = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        //Assigns hardware devices names and values

        GetHardware();
        GetIMU();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


//unfolds here
        UnfoldRobot();
        resetAngle();

        encoderDrive(DRIVE, -20, 3);
        feeder_motor.setPower(1);
        Block_Pickup.setPosition(.4f);
        encoderDrive(DRIVE, -20, 3);

        // encoderDrive(DRIVE, 26, 3);

        encoderDrive(DRIVE, 21, 3);
        top_motor.setPower(1);
        Lift(LiftDirection.DOWN);

        while(!problemChild || bottom_touch.getState())
        {
            if (Top_Sensor_Rear.getState()) {
            } else {
                problemChild = true;
                top_motor.setPower(0);
            }
            if (bottom_touch.getState()) {
            } else {
                Lift(LiftDirection.STOP);
            }
        }
        telemetry.addData("Touch Sensors", "Complete");
        telemetry.update();




        resetAngle();
        //rotate(76, .70);
        rotate(77, 1);


        encoderDrive(DRIVE, 77, 5);
        encoderDrive(DRIVE, -3,2);

        Lift(LiftDirection.UP);
        sleep(150);
        Lift(LiftDirection.STOP);

        resetAngle();
        rotate(79, 1);


        encoderDrive(DRIVE, 10, 1);


        Clamp_Left.setPosition(0.9f);
        Clamp_Right.setPosition(0f);
        sleep(600);

        encoderDrive(DRIVE, 2, 1);

        resetAngle();
        rotate(-10, 1);

        encoderDrive(DRIVE, -23, 3);

       /*front_left.setPower(-1);
       front_right.setPower(1);
       rear_left.setPower(1);
       rear_right.setPower(-1);
       //EncoderDrive(DriveDirection.STRAFE_LEFT, 5000);
       sleep(2000);


        */
        resetAngle();
        rotate(-70,1);

        Lift(LiftDirection.UP);
        Clamp_Left.setPosition(0f);
        Clamp_Right.setPosition(1f);
        sleep(400);
        Lift(LiftDirection.STOP);
        encoderDrive(DRIVE, 13, 2);

        encoderDrive(DRIVE, -10 , 8);

        rotate(180, 1);
        sleep(100);
        feeder_motor.setPower(-1);
        sleep(5000);
        



        telemetry.addData("Path", "Complete");
        telemetry.update();

    }
}


