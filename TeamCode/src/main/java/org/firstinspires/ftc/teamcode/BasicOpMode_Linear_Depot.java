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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import java.lang.annotation.Target;


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

@Autonomous (name="Autonomous Depot", group="Linear Opmode")
public class BasicOpMode_Linear_Depot extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor tl, tr, bl, br, arm,lift, n1;
    private CRServo intake1, intake2, intake3, intake4;
    private GoldAlignDetector detector;
    private OpenGLMatrix lastLocation = null;
    private ElapsedTime runtime2 = new ElapsedTime();

    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Debbie" ,"Lilly");
        telemetry.update();

        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 310; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        tl = hardwareMap.dcMotor.get("top_left_wheel");
        tr = hardwareMap.dcMotor.get("top_right_wheel");
        bl = hardwareMap.dcMotor.get("bottom_left_wheel");
        br = hardwareMap.dcMotor.get("bottom_right_wheel");
        arm = hardwareMap.dcMotor.get("arm");
        lift = hardwareMap.dcMotor.get("lift");
        intake1 = hardwareMap.crservo.get("intake1");
        intake2 = hardwareMap.crservo.get("intake2");
        intake3 = hardwareMap.crservo.get("intake3");
        intake4 = hardwareMap.crservo.get("intake4");
        n1 = hardwareMap.dcMotor.get("n1");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        tr.setDirection(DcMotor.Direction.FORWARD);
        tl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        n1.setPower(-1+Math.random()*2);

        pEncoderMotorRun(0.0018,2390, lift);

        if(runtime.time() >2.5 && lift.isBusy()){
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setPower(0);
        }

        lift.setPower(0);
        sleep(50);
        //turn
        tr.setPower(0);
        tl.setPower(1);
        br.setPower(1);
        bl.setPower(0);
        sleep(150);
        tr.setPower(0);
        tl.setPower(0);
        br.setPower(0);
        bl.setPower(0);

        sleep(100);

        //strafe out
        right_strafe(1,500);



        left_turn(1,1250);

        backwards(1,150);



        sleep(1000);
//left position cube
        if(detector.isFound()){ //left position
            telemetry.addLine("FOUND");

            right_turn(1,500);

            sleep(750);

            right_strafe(1,145);

            backwards(0.6,1000);

            right_turn(1,250);

            backwards(0.5,145);

            sleep(100);

            right_strafe(1,300);

            sleep(100);

            backwards(.5, 150);

            intake1.setPower(1);
            intake2.setPower(-1);
            intake3.setPower(1);
            intake4.setPower(-1);
            sleep(5000);
            intake1.setPower(0);
            intake2.setPower(0);
            intake3.setPower(0);
            intake4.setPower(0);

            sleep(100);

            right_strafe(1,150);

            sleep(250);

            forward(1,1900);

            sleep(250);

            right_strafe(1, 75);

            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            arm.setPower(-0.5);
            sleep(3750);
            arm.setPower(0);
            /*sleep(250);
            //forwards
            tr.setPower(0.5);
            tl.setPower(0.5);
            br.setPower(0.5);
            bl.setPower(0.5);
            sleep(300);
            tr.setPower(0);
            tl.setPower(0);
            br.setPower(0);
            bl.setPower(0);

            sleep(250);
            //turn left
            tr.setPower(1);
            tl.setPower(-1);
            br.setPower(-1);
            bl.setPower(1);
            sleep(575);
            tr.setPower(0);
            tl.setPower(0);
            br.setPower(0);
            bl.setPower(0);

            sleep(250);
            //forwards
            tr.setPower(1);
            tl.setPower(1);
            br.setPower(1);
            bl.setPower(1);
            sleep(100);
            tr.setPower(0);
            tl.setPower(0);
            br.setPower(0);
            bl.setPower(0);

            //strafe left
            tr.setPower(-1);
            tl.setPower(1);
            br.setPower(-1);
            bl.setPower(1);
            sleep(750);
            tr.setPower(0);
            tl.setPower(0);
            br.setPower(0);
            bl.setPower(0);

            sleep(250);
            //forward
            tr.setPower(1);
            tl.setPower(1);
            br.setPower(1);
            bl.setPower(1);
            sleep(2000);
            tr.setPower(0);
            tl.setPower(0);
            br.setPower(0);
            bl.setPower(0);*/

        } else{//middle position cube
            sleep(750);

            forward(1,150);

            sleep(1000);

            if(detector.isFound()){//detector
                right_turn(1,500);

                sleep(100);

                right_strafe(.5,50);

                sleep(50);

                backwards(1,900);

                intake1.setPower(1);
                intake2.setPower(-1);
                intake3.setPower(1);
                intake4.setPower(-1);
                sleep(5500);
                intake1.setPower(0);
                intake2.setPower(0);
                intake3.setPower(0);
                intake4.setPower(0);

                sleep(500);

                left_turn(1,150);

                sleep(200);

                left_strafe(1,250);

                sleep(100);

                forward(1,1950);

                sleep(250);

                left_strafe(1, 100);

                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                arm.setPower(-0.5);
                sleep(3750);
                arm.setPower(0);
                //far right position cube
            } else { //right position

                sleep(550);


                left_strafe(1,100);

                sleep(550);

                forward(1,275);

                sleep(550);

                right_turn(1,345);

                sleep(550);

                backwards(1,1100);

                //left turn
                left_turn(1,135);

                intake1.setPower(1);
                intake2.setPower(-1);
                intake3.setPower(1);
                intake4.setPower(-1);
                sleep(5500);
                intake1.setPower(0);
                intake2.setPower(0);
                intake3.setPower(0);
                intake4.setPower(0);

                right_turn(1, 50);

                left_strafe(1,250);


                forward(1,2000);

                sleep(100);

                left_strafe(1,150);

                sleep(250);

                left_strafe(1, 75);

                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                arm.setPower(-0.5);
                sleep(3600);
                arm.setPower(0);
            }

        }






        // lowers lift
        /*pEncoderMotorRun(0.0018,2423, lift);

        lift.setPower(0);

        pEncoderMotorRun(0.0018, 1120, tl);
        pEncoderMotorRun(0.0018, 1120, br);
        tl.setPower(0);
        bl.setPower(0);

        sleep(100);

        pEncoderMotorRun(0.0018, -1120, tr);
        pEncoderMotorRun(0.0018, 1120, tl);
        pEncoderMotorRun(0.0018, -1120, br);
        pEncoderMotorRun(0.0018, 1120, bl);
        tr.setPower(0);
        tl.setPower(0);
        br.setPower(0);
        bl.setPower(0);

        pEncoderMotorRun(0.0018, 1120, tr);
        pEncoderMotorRun(0.0018, -1120, tl);
        pEncoderMotorRun(0.0018, -1120, br);
        pEncoderMotorRun(0.0018, 1120, bl);
        tr.setPower(0);
        tl.setPower(0);
        br.setPower(0);
        bl.setPower(0);

        sleep(1000);

        //moves forward
       /* tr.setPower(-1);
        tl.setPower(-1);
        br.setPower(-1);
        bl.setPower(-1);
        sleep(675);
        tr.setPower(0);
        tl.setPower(0);
        br.setPower(0);
        bl.setPower(0);

        telemetry.addLine("Last known working, pre-delay");

        sleep(750);

        telemetry.addLine("Last known working, post-delay");

        //outtake
        intake1.setPower(1);
        intake2.setPower(-1);
        intake3.setPower(1);
        intake4.setPower(-1);
        telemetry.addLine("Outtake run");
        sleep(5500);
        intake1.setPower(0);
        intake2.setPower(0);
        intake3.setPower(0);
        intake4.setPower(0);
        telemetry.addLine("Outtake stopped"); */

        /*if(detector.isFound()){
            telemetry.addLine("Found");
            pEncoderMotorRun(0.0018, -1120, tr);
            pEncoderMotorRun(0.0018,1120, tl);
            pEncoderMotorRun(0.0018,-1120,br);
            pEncoderMotorRun(0.0018, 1120, bl);
            tr.setPower(-1);
            tl.setPower(1);
            br.setPower(-1);
            bl.setPower(1);
            sleep(675);

            tr.setPower(0);
            tl.setPower(0);
            br.setPower(0);
            bl.setPower(0);
        }
        else{
            telemetry.addLine("Not Aligned");
            tr.setPower(-0.4);
            tl.setPower(-0.4);
            br.setPower(-0.4);
            bl.setPower(-0.4);
            sleep(800);
            tr.setPower(0);
            tl.setPower(0);
            br.setPower(0);
            bl.setPower(0);

        }*/
        //pArmToLanderFromRest(0.0018, -4080, arm);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        runtime.reset();


    }

    private void pEncoderMotorRun(double kP, double target, DcMotor driveMotor) { //nate
        double error = Math.abs(target - driveMotor.getCurrentPosition());//obtains the robot's position
        double time;
        time = runtime.time();

        while (error > 1 && opModeIsActive()) {//allows the robot to continually operate
            driveMotor.setPower(kP * error);
            error = Math.abs(target - driveMotor.getCurrentPosition());
        }


        /*private void pEncoderDrive(double kP, double brtarget, double bltarget, double trtarget, doubletltarget, DcMotor driveMotor) { //nate
            double error = Math.abs(target - driveMotor.getCurrentPosition());//obtains the robot's position
            double time;
            time = runtime.time();
            while (error > 1 && opModeIsActive()) {//allows the robot to continually operate
                driveMotor.setPower(kP * error);
                error = Math.abs(target - driveMotor.getCurrentPosition());*/
           // }

    }
    private void pArmToLanderFromRest(double kP, double target, DcMotor driveMotor) {
        double error = Math.abs(target - driveMotor.getCurrentPosition());//obtains the arm's position
        while (error > 1) {//allows the robot to continually operate
            driveMotor.setPower(kP * -error);//-power because motor is going backwards toward ground
            error = Math.abs(target - driveMotor.getCurrentPosition());
        }
        arm.setPower(0);
        telemetry.addData("value:", arm.getCurrentPosition());
    }
    private void forward (double power, long time){
        tr.setPower(power);
        tl.setPower(power);
        br.setPower(power);
        bl.setPower(power);
        sleep(time);
        tr.setPower(0);
        tl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    private void backwards (double power, long time){
        tr.setPower(-power);
        tl.setPower(-power);
        br.setPower(-power);
        bl.setPower(-power);
        sleep(time);
        tr.setPower(0);
        tl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    private void right_turn (double power, long time){
        tr.setPower(-power);
        tl.setPower(power);
        br.setPower(power);
        bl.setPower(-power);
        sleep(time);
        tr.setPower(0);
        tl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    private void left_turn(double power, long time){
        tr.setPower(power);
        tl.setPower(-power);
        br.setPower(-power);
        bl.setPower(power);
        sleep(time);
        tr.setPower(0);
        tl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    private void right_strafe(double power, long time){
        tr.setPower(-power);
        tl.setPower(power);
        br.setPower(-power);
        bl.setPower(power);
        sleep(time);
        tr.setPower(0);
        tl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    private void left_strafe(double power, long time){
        tr.setPower(power);
        tl.setPower(-power);
        br.setPower(power);
        bl.setPower(-power);
        sleep(time);
        tr.setPower(0);
        tl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }


}




