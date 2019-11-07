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
import com.qualcomm.robotcore.util.RobotLog;

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
@Autonomous(name="FoundationAndParkOpMode", group="Linear Opmode")
//@Disabled
public class NerdFoundationAndParkOpMode extends LinearOpMode {
    private NerdBOT myNerdBOT ;

    private  double speed = 0.4;
    boolean debugFlag = true;
    private NerdArmMove Arm;

    //Things to be changed depending on dominant alliance partner (for parking and distance to foundation)
    private final long SLEEP_TIME = 0;
    private final double X_DISTANCE_TO_FOUNDATION = 24.0;
    private final double Y_DISTANCE_TO_FOUNDATION = 0.0;
    private final double Z_ANGLE_FOUNDATION = 0.0;
    private final double Y_DISTANCE_UP_TO_FOUNDATION = 0.0;
    private final long SLEEP_TIME2 = 0;
    private final double X_DISTANCE_TO_PARKING = 36.0;
    private final double Y_DISTANCE_TO_PARKING = 0.0;
    private final double Z_ANGLE_PARKING = 0.0;
    private final double FORWARD_ON_PARKING_LINE = 0.0;

    @Override
    public void runOpMode() {
        //Create a NerdBOT object
        myNerdBOT = new NerdBOT(this);
        Arm = new NerdArmMove(this);

        myNerdBOT.setDebug(debugFlag);

            //Initialize Hardware
        myNerdBOT.initializeHardware();
        Arm.initHardware();
        //Initialize the PID Calculators
        myNerdBOT.initializeXPIDCalculator(0.0025, 0.0, 0.0, debugFlag);
        myNerdBOT.initializeYPIDCalculator(0.0025, 0.0, 0.0,debugFlag);
        myNerdBOT.initializeZPIDCalculator(0.015, 0.000, 0.0,debugFlag);
        myNerdBOT.initializeTurnPIDCalculator(0.015, 0.000, 0.02535,debugFlag);
        //Set Min and Max Speed - Optional (default min=0.1, max=0.6 if not changed below)
        myNerdBOT.setMinMaxSpeeds(0.0,0.5);


        telemetry.addData("Init", "Completed");
        telemetry.update();


        waitForStart();
        sleep(SLEEP_TIME);


        //UNITS ARE IN INCHES
        if (debugFlag)
            RobotLog.d("NerdSampleOpMode - Run1");


        myNerdBOT.nerdPidDrive( speed, X_DISTANCE_TO_FOUNDATION, Y_DISTANCE_TO_FOUNDATION, Z_ANGLE_FOUNDATION);
        myNerdBOT.nerdPidDrive( speed, 0.0, Y_DISTANCE_UP_TO_FOUNDATION, 0.0);
        Arm.UseTheForce();
        myNerdBOT.nerdPidDrive(speed, 0.0, -36.0, 0);
        Arm.ArmLoop(-10,7, 0.5, 0.5);
        myNerdBOT.nerdPidDrive(speed, X_DISTANCE_TO_PARKING, Y_DISTANCE_TO_PARKING, Z_ANGLE_PARKING);
        myNerdBOT.nerdPidDrive(speed, 0.0, FORWARD_ON_PARKING_LINE, 0);






    }
}