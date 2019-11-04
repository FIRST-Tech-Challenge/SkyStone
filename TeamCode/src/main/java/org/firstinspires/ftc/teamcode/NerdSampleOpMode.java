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
@Autonomous(name="NerdSampleOpMode", group="Linear Opmode")
//@Disabled
public class NerdSampleOpMode extends LinearOpMode {
    private NerdBOT myNerdBOT ;
    private ArmMove Arm;
    private  double speed = 0.4;
    private double Skystone_Position = 0;
    private double position_run3_x = 81.0;
    private double offset_x_run3 = 0;
    private  double run3_x = 0;
    boolean debugFlag = true;
    static private VuforiaFindCase VFC;
    @Override
    public void runOpMode() {
        //Create a NerdBOT object
        myNerdBOT = new NerdBOT(this);
        Arm = new ArmMove(this);
        myNerdBOT.setDebug(debugFlag);
        //Initialize Hardware
        myNerdBOT.initializeHardware();
        Arm.initHardware();
        //Initialize the PID Calculators
        myNerdBOT.initializeXPIDCalculator(0.0025, 0.0, 0.0, debugFlag);
        myNerdBOT.initializeYPIDCalculator(0.0025, 0.0, 0.0,debugFlag);
        myNerdBOT.initializeZPIDCalculator(0.015, 0.000, 0.0,debugFlag);
        myNerdBOT.initializeTurnPIDCalculator(0.024, 0.000, 1.85,debugFlag);
        //Set Min and Max Speed - Optional (default min=0.1, max=0.6 if not changed below)
        myNerdBOT.setMinMaxSpeeds(0.0,0.5);
        waitForStart();
        //UNITS ARE IN INCHES
        if (debugFlag)
            RobotLog.d("NerdSampleOpMode - Run1");
        myNerdBOT.nerdPidDrive( speed, 0.0, 10.0, 0.0);
        VFC = new VuforiaFindCase(this);
        Skystone_Position = VFC.vuforia();
        telemetry.addData("Position Case",Skystone_Position );
        telemetry.update();
        if (debugFlag)
            RobotLog.d("NerdSampleOpMode - Run2");

        telemetry.addData("before", "arm");
        telemetry.update();

        Arm.ArmLoop(-135,7);

        telemetry.addData("after", "arm");
        telemetry.update();


        if (Skystone_Position == 3) {
            myNerdBOT.nerdPidDrive(speed, 8.0, 13.5, 0.0);
            offset_x_run3 = 8.0;
            sleep(2000);
        }
        else if (Skystone_Position == 2) {
            myNerdBOT.nerdPidDrive(speed, 0.0, 13.5, 0.0);
            offset_x_run3 = 0.0;
            sleep(2000);
        }
        else if (Skystone_Position == 1) {
            myNerdBOT.nerdPidDrive(speed, -8.0, 13.5, 0.0);
            offset_x_run3 = -8.0;
            sleep(2000);
        }
        else
        {
        }



          Arm.ArmLoop(-135,140);
          sleep(500);
           Arm.ArmLoop(-10,7);
        if (debugFlag)
            RobotLog.d("NerdSampleOpMode - Run3");
        myNerdBOT.setMinMaxSpeeds(0.0,0.7); // Go faster when going longer distance.
        run3_x = (position_run3_x+offset_x_run3);
        myNerdBOT.nerdPidDrive( speed, -run3_x, -5.0, 0.0);
        myNerdBOT.setMinMaxSpeeds(0.0,0.5);
        if (debugFlag)
            RobotLog.d("NerdSampleOpMode - Run4");
        myNerdBOT.nerdPidDrive( speed, 0.0, 15.0, 0.0);

        Arm.ArmLoop(-60,135);
        Arm.ArmLoop(-125,143);
        Arm.ArmLoop(-135,7);

        if (debugFlag)
            RobotLog.d("NerdSampleOpMode - Completed");
    }
}