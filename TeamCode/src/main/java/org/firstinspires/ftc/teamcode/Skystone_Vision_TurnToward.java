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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;


/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

@Autonomous(name = "Skystone_Vision_TurnToward", group = "Linear Opmode")
//@Disabled
public class Skystone_Vision_TurnToward extends BaseAutoOpMode {
    Comparator<Recognition> compareByAngle = new Comparator<Recognition>() {
        @Override
        public int compare(Recognition o1, Recognition o2) {
            return (int) Math.round(Math.abs(o1.estimateAngleToObject(AngleUnit.DEGREES)) - (Math.abs(o2.estimateAngleToObject(AngleUnit.DEGREES))));
        }
    };

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        GetIMU();
        GetHardware();
        InitVision();


        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            VisionTargetTfod(); //continue to update vision
        }
        runtime.reset();
        resetAngle();

        //EncoderDrive(DriveDirection.BACKWARD, 100);
        int movements = 0;


        while (!isStopRequested()) {
            List<Recognition> Skystones = VisionTargetTfod();

            //figure out the stone closest to the front of our robot
            Collections.sort(Skystones, compareByAngle);

            if (Skystones.size() > 0) //stones detected
            {
                telemetry.addData("Status", "Detected " + Skystones.size() + " Skystones");


                //first stone in the list should be the closest
                Recognition stone = Skystones.get(0);

                telemetry.addData("Object 0", stone.getLabel());

                double angleToStone = stone.estimateAngleToObject(AngleUnit.DEGREES);
<<<<<<< HEAD
                if (Math.abs(angleToStone) < 5) {
=======
                telemetry.addData("Angle", angleToStone);

                if( Math.abs(angleToStone) < 5)
                {
>>>>>>> a28f47ba9c4df9967b120511102568830b082e3b
                    Drive(DriveDirection.BACKWARD);
                    sleep(1000);
                    StopAllDrive();
                } else {
                    if (angleToStone < 0) //turn left, but we're backwards
                    {
                        Drive(DriveDirection.RIGHT);
                    } else //turn right
                    {
                        Drive(DriveDirection.LEFT);
                    }
                }

                telemetry.update();

            } else {
                telemetry.addData("Status", "Not detected");
                telemetry.addData("Movements", movements);
                if(movements <= 30)
                {
                    movements++;
                    EncoderDrive(DriveDirection.BACKWARD, 100);
                }



                telemetry.update();
            }
        }


    }
}

