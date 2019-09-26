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

@Autonomous (name="Autonomous Crater_New", group="Linear Opmode")
public class BasicOpMode_Linear_Crater_New extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor tl, tr, bl, br, arm, lift, n1;
    private CRServo intake1, intake2, intake3, intake4;
    private GoldAlignDetector detector;
    private OpenGLMatrix lastLocation = null;
    private ElapsedTime runtime2 = new ElapsedTime();

    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Debbie", "Lilly");
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
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        n1.setPower(-1 + Math.random() * 2);

        pEncoderMotorRun(0.0018, 2390, lift);

        if (runtime.time() > 2.5 && lift.isBusy()) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setPower(0);
        }

        lift.setPower(0);

        sleep(50);
        //turn right
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

        if(detector.isFound()) { //left position
            //turn left
           left_turn(1,500);

            sleep(1000);

            //strafe left
           /* tr.setPower(1);
            tl.setPower(-1);
            br.setPower(1);
            bl.setPower(-1);
            sleep(120);
            tr.setPower(0);
            tl.setPower(0);
            br.setPower(0);
            bl.setPower(0);

            sleep(500);*/

            //straight
           forward(.5,500);

            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            arm.setPower(-0.5);
            sleep(3750);
            arm.setPower(0);

        } else {//middle position
            sleep(750);
            //forward
           forward(1,150);

            sleep(1000);

            if (detector.isFound()) {
                //turn left
               left_turn(1,500);

                sleep(1000);

                //strafe left
                left_strafe(1,75);

                sleep(500);

                //straight
                forward(.5,500);

                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                arm.setPower(-0.5);
                sleep(3750);
                arm.setPower(0);
            } else { //right position

                sleep(550);

                //left Strafe
                left_strafe(1,100);

                sleep(550);
                //forward
                forward(1,275);

                sleep(550);
                //turn left
                left_turn(1,500);

                sleep(500);

                //strafe left
                left_strafe(1,125);

                sleep(750);
                //straight
               forward(.5,500);

                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                arm.setPower(-0.5);
                sleep(3750);
                arm.setPower(0);
            }
        }

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