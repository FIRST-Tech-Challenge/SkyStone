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
//test for Slack integration

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


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

@Autonomous(name="Nightly", group="Linear Opmode")
//@Disabled
public class Nightly extends LinearOpMode {

    //Driving Motors
    private DcMotor lf = null; //"lf" stands for left front and so on.
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private Servo foundation = null;
    private Servo release = null;
    private DcMotor elevator = null;

    //Sensors
    private ColorSensor sensorColor = null;
    private DistanceSensor sensorDistance = null;

    @Override
    public void runOpMode() { //throws interrupted exception
        //initialize required driving motors
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        foundation = hardwareMap.get(Servo.class, "foundation");
        release = hardwareMap.get(Servo.class, "release");
        elevator = hardwareMap.get(DcMotor.class, "elevator");

        //initialize sensors
        // get a reference to the color sensor.
//        sensorColor = hardwareMap.get(ColorSensor.class, "SCD");
//
//        // get a reference to the distance sensor that shares the same name.
//        sensorDistance = hardwareMap.get(DistanceSensor.class, "SCD");


        //declare final variables here
        //motor variables
        final int FULL_POWER_MOTOR = 1;
        final int STOP_POWER_MOTOR = 0;

        //Sensors
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        boolean object = false;
        //true = block

        //Motor Direction settings
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.REVERSE);

        //we wil try to use encoders
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//        while (opModeIsActive()){
//
//        }
        //Methods
//        DriveForward(0.5, 1794);      //one revolution is 1120 Andymark
        //tetrix and modern robotics is 1440
        //about a 200mm circumference
//        StrafeRight(0.8, 1769);
//        Elevate(1, 5600);





//        StrafeRight(0.5, 1700);
//        DriveForward(0.3, 1800);
////        StrafeLeft(0.5,3756);
//        //while (foundation.getPosition()!=0.3)
//            foundation.setPosition(1);
//        sleep(2000);
////        StrafeLeft(0.5,1500);
////        StrafeRight(0.5, 3796);
//        DriveForward(-0.3, -2000);
//        foundation.setPosition(0);
//        sleep(500);
//        StrafeLeft(0.5,4756);
//        DriveForward(0.3, 2000);


//        StrafeRight(0.5, 2200);
//        DriveForward(0.3, 2300);
////        TurnRight(0.5, 150);
////        DriveForward(0.2, 200);
//
////        StrafeLeft(0.5,3756);
//        //while (foundation.getPosition()!=0.3)
//        foundation.setPosition(1);
//        sleep(2000);
////        StrafeLeft(0.5,1500);
////        StrafeRight(0.5, 3796);
//        DriveFrward(-0.3, -3500);
//        foundation.setPosition(0);
//        sleep(500);
//        StrafeLeft(0.5,1492);
//
////        StrafeRight(0.5,2000);
//
////        DriveForward(0.3, 2300);
//
//        sleep(1000);
//        DriveForward(0.3, 1000);
//        release.setPosition(-1.0);
//        DriveForward(-0.3, -2000);
//        foundation.setPosition(0.5);
//        sleep(150);



////////////////////////////////////////






        StrafeRight(0.5, 1900);
        DriveForward(0.3, 2300);
//        TurnRight(0.5, 150);
        DriveForward(0.2, 200);

//        StrafeLeft(0.5,3756);
        //while (foundation.getPosition()!=0.3)
        foundation.setPosition(1);
        sleep(2000);
//        StrafeLeft(0.5,1500);
//        StrafeRight(0.5, 3796);
        DriveForward(-0.3, -2300);
        foundation.setPosition(0);
        sleep(500);
        StrafeLeft(0.5,1492);

//        StrafeRight(0.5,2000);

//        DriveForward(0.3, 2300);

        sleep(1000);
        DriveForward(0.3, 1000);
        release.setPosition(-1.0);
        DriveForward(-0.3, -2000);
        foundation.setPosition(0.5);
        sleep(150);





















//
//        TurnRight(0.5, -750);
//        DriveForward(1, 1794);
//        Elevate(-1, -5500);
//        while (true) Sensation(object,SCALE_FACTOR, hsvValues);

        // run until the end of the match (driver presses STOP)
        // Now we're done, yee

    }

    public void DriveForward(double power, int distance)//Drive Forward
    {
        //resets encoder values
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sets Target position
        lf.setTargetPosition(distance);
        lb.setTargetPosition(distance);
        rf.setTargetPosition(distance);
        rb.setTargetPosition(distance);

        //sets to runs to position
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //runs
        lf.setPower(power);
        lb.setPower(power);
        rf.setPower(power);
        rb.setPower(power);
        while (lf.isBusy()&&lb.isBusy()&&rf.isBusy()&&rb.isBusy()){                 //RED FLAG CHECK THIS THING!!!!
            //waits for all motors to stop
        }
        DF(0);  //sets power to 0

        //resets mode
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void DriveFrward(double power, int distance)//Drive Forward
    {
        //resets encoder values
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sets Target position
        lf.setTargetPosition(distance);
        lb.setTargetPosition(distance);
        rf.setTargetPosition(distance);
        rb.setTargetPosition(distance);

        //sets to runs to position
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //runs
        lf.setPower(power*2.5);
        lb.setPower(power*2.5);
        rf.setPower(power);
        rb.setPower(power);
        while (lf.isBusy()&&lb.isBusy()&&rf.isBusy()&&rb.isBusy()){                 //RED FLAG CHECK THIS THING!!!!
            //waits for all motors to stop
        }
        DF(0);  //sets power to 0

        //resets mode
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void DF(double power){           //Method with no end, DO NOT USE UNLESS NECESSARY
        lf.setPower(power);
        lb.setPower(power);
        rf.setPower(power);
        rb.setPower(power);
    }
    public void swerve (long m,double power,String d){
        if (d.equals("back")) {
            lb.setPower(power);
            rb.setPower(-power);
            sleep(m);
            lb.setPower(0);
            rb.setPower(0);
        } else{
            lf.setPower(power);
            rf.setPower(-power);
            sleep(m);
            lf.setPower(0);
            rf.setPower(0);
        }
    }
    public void Elevate(double power, int distance){
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setTargetPosition(distance);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(power);
        while (elevator.isBusy()){

        }
        elevator.setPower(0);

    }
    public void TurnRight(double power, int distance)//Drive Forward
    {

        //resets encoder values
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sets Target position
        lf.setTargetPosition(distance);
        lb.setTargetPosition(distance);
        rf.setTargetPosition(-distance);
        rb.setTargetPosition(-distance);

        //sets to runs to position
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //runs
        lf.setPower(power);
        lb.setPower(power);
        rf.setPower(-power);
        rb.setPower(-power);
        while (lf.isBusy()&&lb.isBusy()&&rf.isBusy()&&rb.isBusy()){
            //waits for all motors to stop
        }
        DF(0);  //sets power to 0 to stop robot

        //resets mode
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //resets encoder values
//        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        lf.setPower(-power);
//        lb.setPower(-power);
//        rf.setPower(power);
//        rb.setPower(power);
    }
    public void TurnLeft(double power)//Drive Forward
    {
        lf.setPower(power);
        lb.setPower(power);
        rf.setPower(-power);
        rb.setPower(-power);
    }
    public void StrafeRight(double power, int distance)//Drive Forward
    {
        //resets encoder values
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sets Target position
        lf.setTargetPosition(distance);
        lb.setTargetPosition(-distance);
        rf.setTargetPosition(-distance);
        rb.setTargetPosition(distance);

        //sets to runs to position
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //runs
        lf.setPower(power);
        lb.setPower(-power);
        rf.setPower(-power);
        rb.setPower(power);
//        boolean a,b,c,d,e;
//        a=false;
//        b=false;
//        c=false;
//        d=false;
//        e=true;
//        while (e){
//            if (!lf.isBusy()){
//                lf.setPower(0);
//                a=true;
//            }
//            if (!lb.isBusy()){
//                lb.setPower(0);
//                b=true;
//            }
//            if (!rf.isBusy()){
//                rf.setPower(0);
//                c=true;
//            }
//            if (!rb.isBusy()){
//                rb.setPower(0);
//                d=true;
//            }
//            if (a&&b&&c&&d)
//                e=false;
//        }
////        while (lf.isBusy()&&lb.isBusy()&&rf.isBusy()&&rb.isBusy()){
////            //waits for all motors to stop
////        }
////        DF(0);  //sets power to 0 to stop robot

        while (lf.isBusy()&&lb.isBusy()&&rf.isBusy()&&rb.isBusy()){
            //waits for all motors to stop
        }
        DF(0);  //sets power to 0 to stop robot

        //resets mode
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void StrafeLeft(double power, int distance)//Drive Forward
    {
        //resets encoder values
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sets Target position
        lf.setTargetPosition(-distance);
        lb.setTargetPosition(distance);
        rf.setTargetPosition(distance);
        rb.setTargetPosition(-distance);

        //sets to runs to position
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //runs
        lf.setPower(-power);
        lb.setPower(power);
        rf.setPower(power);
        rb.setPower(-power);
//        boolean a,b,c,d,e;
//        a=false;
//        b=false;
//        c=false;
//        d=false;
//        e=true;
//        while (e){
//            if (!lf.isBusy()){
//                lf.setPower(0);
//                a=true;
//            }
//            if (!lb.isBusy()){
//                lb.setPower(0);
//                b=true;
//            }
//            if (!rf.isBusy()){
//                rf.setPower(0);
//                c=true;
//            }
//            if (!rb.isBusy()){
//                rb.setPower(0);
//                d=true;
//            }
//            if (a&&b&&c&&d)
//                e=false;
//        }
        while (lf.isBusy()&&lb.isBusy()&&rf.isBusy()&&rb.isBusy()){
            //waits for all motors to stop
        }
        DF(0);  //sets power to 0 to stop robot

        //resets mode
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void Sensation(boolean object , double SCALE_FACTOR, float hsvValues[]){
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        if (hsvValues[0]>100)
            object = false;
        telemetry.addData("Mineral", "Silver");
        if (hsvValues[0]<100)
            object = true;
        telemetry.addData("Mineral", "Gold");

    }
//    public void Alldir(double power, double degrees, int distance){       //Do not worry about this
//        //resets encoder values
//        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        //sets Target position
//        lf.setTargetPosition(distance);
//        lb.setTargetPosition(distance);
//        rf.setTargetPosition(distance);
//        rb.setTargetPosition(distance);
//
//        //sets to runs to position
//        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        //runs
//        lf.setPower(power);
//        lb.setPower(power);
//        rf.setPower(power);
//        rb.setPower(power);
//        while (lf.isBusy()&&lb.isBusy()&&rf.isBusy()&&rb.isBusy()){
//            //waits for all motors to stop
//        }
//        DF(0);  //sets power to 0
//
//        //resets mode
//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
}
