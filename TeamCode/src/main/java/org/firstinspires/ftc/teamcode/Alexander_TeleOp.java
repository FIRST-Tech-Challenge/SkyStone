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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.*;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.I2cAddr;
//import com.qualcomm.robotcore.hardware.I2cDevice;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Alexander_TeleOp", group="Linear OpMode")
//@Disabled
public class Alexander_TeleOp extends LinearOpMode {


    private DcMotor getNewMotor(String motorName) { //these could be made generic using type notation
        try {
            return (hardwareMap.get(DcMotor.class, motorName));
        } catch (Exception e) {
            telemetry.addData("MOTOR: "+motorName, "   offline");
            telemetry.update();
            return (null);
        }
    }

    //Speed Factors for Fast/Slow Mode
    private double speedFactor = 1.0; //default full speed
    private ElapsedTime runtime = new ElapsedTime();
    private double currentLoopTime = 0.0;
    private double lastLoopTime = 0.0;

    //Driving Motors
    private DcMotor frontLeft = null; 
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    //Attachment Motors
    private DcMotor collectorLeft = null;
    private DcMotor collectorRight = null;
    private DcMotor linearSlide = null;
    private DcMotor clamps = null;


    //other variables
    private static boolean tele = true; //show telemetry



    //Our software coach from last year helped us with this method that uses trigonometry to operate mecanum wheels
    private void mecanumMove(double leftStickX, double leftStickY, double rightStickX) {


        double distanceFromCenter = Math.sqrt(leftStickY * leftStickY + leftStickX * leftStickX);  // might be leftStickY * leftStickX This double uses the pythagoren theorem to find  out the distance from the the joystick center

        double robotAngle = Math.atan2(-1 * leftStickY, leftStickX) - Math.PI / 4;

        final double frontLeftPower = distanceFromCenter * Math.cos(robotAngle) + rightStickX;    //Multiplies the scaling of the joystick to give different speeds based on joystick movement
        final double frontRightPower = distanceFromCenter * Math.sin(robotAngle) - rightStickX;
        final double backLeftPower = distanceFromCenter * Math.sin(robotAngle) + rightStickX;
        final double backRightPower = distanceFromCenter * Math.cos(robotAngle) - rightStickX;

        if(frontLeft != null)
            frontLeft.setPower(frontLeftPower);
        if(frontRight != null)
            frontRight.setPower(frontRightPower);
        if(backLeft != null)
            backLeft.setPower(backLeftPower);
        if(backRight != null)
         backRight.setPower(backRightPower);
    }
    private void Collector(boolean onOff)
    {
        if(onOff)
        {

            if(collectorLeft != null) {
                collectorLeft.setPower(1.0);
            }
            if(collectorRight != null) {
                collectorRight.setPower(-1.0);
            }

        }
        else
        {
            if(collectorLeft != null) {
                collectorLeft.setPower(0.0);
            }
            if(collectorRight != null) {
                collectorRight.setPower(0.0);
            }
        }

    }

    private void SkystonePositioner(double rightStickY)
    {
        if(linearSlide != null) {
            linearSlide.setPower(rightStickY);
        }
    }

     private void moveClamps(boolean dPadUp, boolean dPadDown)
    {
        if (dPadUp) {
            if (clamps != null) {
                clamps.setPower(1.0);
            }
        }
        else if (dPadDown)
        {
            if (clamps != null) {
                clamps.setPower(-1.0);
            }
        }
    }
    @Override
    public void runOpMode() throws InterruptedException{

        
            telemetry.addData("Droid", "Robot");
        


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);

        //Above line is commented out because Hardware map is used for accessories such as attachment sensors/servos/motors

        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Say", "Hello Driver");

        telemetry.addData("Status", "Initialized");                 //Telemetry is the messages displayed on phone
        telemetry.update();

        //initialize required driving motors
        frontLeft = getNewMotor("frontLeft");
        frontRight = getNewMotor("frontRight");
        backLeft = getNewMotor("backLeft");
        backRight = getNewMotor("backRight");

        if(frontLeft != null)
            frontLeft.setDirection(DcMotor.Direction.FORWARD);           // This makes the front of the robot the side with the block intake!!!!!
        if(frontRight != null)                                          // This makes the front of the robot the side with the block intake!!!!!
            frontRight.setDirection(DcMotor.Direction.REVERSE);
        if(backLeft != null)
            backLeft.setDirection(DcMotor.Direction.FORWARD);
        if(backRight != null)
            backRight.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();    //resets runtime()

        currentLoopTime = runtime.time();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            lastLoopTime = currentLoopTime;
            currentLoopTime = runtime.time();

            //gamepad1
            double forwardBack = gamepad1.left_stick_y;
            double leftRight = gamepad1.left_stick_x;
            double Rotate = gamepad1.right_stick_x;
            boolean slowMode = (gamepad1.left_trigger > 0.2);


            //gamepad2
            boolean collectorOn = gamepad2.a;
            double StoneUpDown = gamepad2.right_stick_y;
            boolean clampsIn = gamepad2.dpad_up;
            boolean clampsOut = gamepad2.dpad_down;



            if (slowMode) {
                speedFactor = 0.8;
            } else {
                speedFactor = 1.0;
            }

            double leftStickY = forwardBack * speedFactor;
            double leftStickX = leftRight * speedFactor;
            double rightStickX = Rotate * speedFactor;

            //Methods
            mecanumMove(leftStickX, leftStickY, rightStickX);
            Collector(collectorOn);
            SkystonePositioner(StoneUpDown);
            moveClamps(clampsIn, clampsOut);


            //Telemetry
            if (tele) telemetry.addData("Status", "Run Time: " + runtime.toString());
            if (tele) telemetry.update();

        }
    }
}