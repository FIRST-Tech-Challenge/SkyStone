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
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;



//import lines go here. This is just for the program and not for the robot.

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="NerdSampleOpMode", group="Linear Opmode")
//@Disabled
public class NerdSampleOpMode extends LinearOpMode {

   private NerdBOT myNerdBOT ;
   private  double speed = 0.4;

    @Override
    public void runOpMode() {

        //Initialize Hardware

        myNerdBOT = new NerdBOT(this);

        myNerdBOT.initializeHardware();

        //Initialize the PID Calculators

        // Use these for Competition Robot
//        myNerdBOT.initializeXPIDCalculator(0.002,0.0,1.1);
//        myNerdBOT.initializeYPIDCalculator(0.002,0.0,1.1);
//        myNerdBOT.initializeZPIDCalculator(0.008,0.0,1.6);
//        myNerdBOT.initializeTurnPIDCalculator(0.009,0.0,0.9);

        //Use these for Quad Bot
        myNerdBOT.initializeXPIDCalculator(0.0005, 0.005, 0.5);
        myNerdBOT.initializeYPIDCalculator(0.0005, 0.005, 0.5);
        myNerdBOT.initializeZPIDCalculator(0.036, 0.08, 0.0);
        myNerdBOT.initializeTurnPIDCalculator(0.024, 0.000, 1.85);

        //Set Min and Max Speed - Optional (default min=0.1, max=0.6 if not changed below)

        myNerdBOT.setMinMaxSpeeds(0.1,0.6);

        waitForStart();

        //UNITS ARE IN INCHES
        RobotLog.d("NerdSampleOpMode - Starting nerdPidDrive yDistance = 25");
        myNerdBOT.nerdPidDrive( speed, 0.0, 20.0, 0.0);
        sleep(1000);
        RobotLog.d("NerdSampleOpMode -  Angle After nerdPidDrive : %f", myNerdBOT.getZAngleValue());

        RobotLog.d("NerdSampleOpMode - Starting nerdPidDrive xDistance = 25");
        myNerdBOT.nerdPidDrive( speed, 20.0, 0.0, 0.0);
        sleep(1000);
        RobotLog.d("NerdSampleOpMode -  Angle After nerdPidDrive : %f", myNerdBOT.getZAngleValue());

        RobotLog.d("NerdSampleOpMode - Starting nerdPidDrive yDistance = -25");
        myNerdBOT.nerdPidDrive(speed, 0.0, -20.0, 0.0);
        sleep(1000);
        RobotLog.d("NerdSampleOpMode -  Angle After nerdPidDrive : %f", myNerdBOT.getZAngleValue());

        RobotLog.d("NerdSampleOpMode - Starting nerdPidDrive xDistance = -20.0, yDistance = 20.0 ");
        myNerdBOT.nerdPidDrive( speed, -20.0, 20.0, 0.0);
        sleep(1000);
        RobotLog.d("NerdSampleOpMode - Angle After nerdPidDrive : %f", myNerdBOT.getZAngleValue());

        RobotLog.d("NerdSampleOpMode - Starting nerdPidTurn targetAngle = 90.0");
        myNerdBOT.nerdPidTurn(0.2, 90.0, false);
        sleep(1000);
        RobotLog.d("NerdSampleOpMode - FinalAngle After nerdPidTurn : %f", myNerdBOT.getZAngleValue());

        RobotLog.d("NerdSampleOpMode - Completed");


    }



}




