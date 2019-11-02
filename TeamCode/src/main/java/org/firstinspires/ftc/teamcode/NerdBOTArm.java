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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
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

public class NerdBOTArm {

    private boolean debugFlag=false;

    private DcMotor leftMotor;
   private DcMotor rightMotor;

    LinearOpMode opmode;

    private ElapsedTime FPIDTime = new ElapsedTime();

    private ElapsedTime RPIDTime = new ElapsedTime();

    private ElapsedTime PIDTime = new ElapsedTime();

    private double RPrevError = 0;
    private double FPrevError = 0;

    private double RTotalError = 0;
    private double FTotalError = 0;


    private double RSpeed = 0;
    private double FSpeed = 0;

    public double REV = -14;
    public double FEV = 7;

    public double FkP = 0.03; //0.03, 0.015
    public double FkI = 0; //0.02
    public double FkD = 0;//0.001

    public double RkP = 0.005; //0.15, 0.005
    public double RkI = 0; //0.03
    public double RkD = 0;//0.0008

    private double MSR = 0.5;
    private double MSF = 0.5;

    private double MaxSpeedR = MSR;
    private double MaxSpeedF = MSF;

    public DcMotor frontMotor;
    public DcMotor rearMotor;

    //Initial Speed for Robot to run


    private HardwareMap hardwareMap;

    public enum ArmAction{
        PICKUP,
        GRAB,
        DROP,
        HOME,
        DROP2,
        FOUNDATION
    }



    /**Constructor to create NerdBOT object
     *
     * Creates a new NerdBOT object and assigns the hardwareMap provided by caller
     * @param   opmode  Hardware Map provided by the calling OpMode.
     *
     */

    public NerdBOTArm(LinearOpMode opmode){
        this.opmode = opmode;
        this.hardwareMap = opmode.hardwareMap;
    }


    public void nerdPIDArmPerformTask(double EV, double TPos, double kP, double kI, double kD, boolean motor) {

        double DError = 0;
        int DBanMin = -1;
        int DBanMax = 1;
        int MaxError = 10;
        double error = 0;
        double speed = 0;
        double TotalError = 0;
        double PrevError = 0;
        double MaxSpeed = 0;




        if(motor) {
            TotalError = RTotalError;
            PrevError = RPrevError;
            PIDTime = RPIDTime;
            MaxSpeed = MaxSpeedR;
        } else {
            TotalError = FTotalError;
            PrevError = FPrevError;
            PIDTime = FPIDTime;
            MaxSpeed = MaxSpeedF;
        }


        //calculate error (Proportional)
        error = TPos - EV;

        //Calculate Total error (Integral)
        TotalError = (error * PIDTime.seconds()) + TotalError;

        //do deadban
        if(DBanMax > error && error > DBanMin) {
            error = 0;
            TotalError = 0;
        }

        //calculate delta error (Derivative)
        DError = (error - PrevError) / PIDTime.seconds();

        //reset elapsed timer
        PIDTime.reset();

        //Max total error
        if(Math.abs(TotalError) > MaxError) {


            if(TotalError > 0) {
                TotalError = MaxError;
            } else {
                TotalError = -MaxError;
            }

        }



        //Calculate final speed
        speed = (error * kP) + (TotalError * kI) + (DError * kD);


        //Make sure speed is no larger than MaxSpeed
        if(Math.abs(speed) > MaxSpeed) {
            if(speed > 0) {
                speed = MaxSpeed;
            } else {
                speed = -MaxSpeed;
            }
        }
        PrevError = error;

        if(motor) {
            RSpeed = speed;
            RPrevError = PrevError;
            RTotalError = TotalError;
        } else {
            FSpeed = speed;
            FPrevError = PrevError;
            FTotalError = TotalError;
        }
        //set previous error to error


        //add telemetry

    }

    public void setUpArmAction(ArmAction action) {
        switch (action) {
            case PICKUP: { //pick up
                REV = -125;
                FEV = 25;
                MaxSpeedR = 0.5;
                MaxSpeedF = 0.5;
            }
            case GRAB: { //grab
                REV = -135;
                FEV = 143;
                MaxSpeedR = 0.5;
                MaxSpeedF = 0.5;
            }
            case DROP:{ //drop
                REV = -45;
                FEV = 120;
                MaxSpeedR = 0.2;
                MaxSpeedF = 0.6;
            }
            case HOME:{ //home
                REV = -10;
                FEV = 25;
                MaxSpeedR = 0.6;
                MaxSpeedF = 0.2;
            }
            case DROP2: { //drop 2
                REV = -50;
                FEV = 110;
                MaxSpeedR = 0.2;
                MaxSpeedF = 0.6;
            }
            case FOUNDATION:{ //foundation
                REV = -145;
                FEV = 25;
                MaxSpeedR = 1;
                MaxSpeedF = 1;
            }
        }
    }

    public void initializeArmMotors(){

        this.frontMotor = hardwareMap.get(DcMotor.class, "frontMotor");
        this.rearMotor = hardwareMap.get(DcMotor.class, "rearMotor");
    }

    public  void nerdBOTArmSetMotorPowers(){
            this.rearMotor.setPower(this.RSpeed);
            this.frontMotor.setPower(this.FSpeed);
    }


}




