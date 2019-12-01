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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
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

@Autonomous(name="BlueAutonomous", group="Linear OpMode")
//@Disabled
public class BlueAutonomous extends LinearOpMode {


    private DcMotor getNewMotor(String motorName) { //these could be made generic using type notation
        try {
            return (hardwareMap.get(DcMotor.class, motorName));
        } catch (Exception e) {
            telemetry.addData("MOTOR: " + motorName, "   offline");
            telemetry.update();
            return (null);
        }
    }


    //Driving Motors
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    //Attachment Motors
    private DcMotor collectorLeft = null;
    private DcMotor collectorRight = null;
    private DcMotor linearSlide = null;

    //Attachment Servos
    private CRServo clamp = null;
    private Servo rotation = null;
    private Servo foundation = null;
    private Servo release = null;

    //Sensors
    private ColorSensor blueColorSensor = null;
    private ColorSensor redColorSensor = null;

    // Other
    int skystonePosition; // Can equal 1, 2, or 3. This corresponds to the A, B and C patterns.
    boolean Stone1isBlack; // Is the first stone black?
    boolean Stone2isBlack; // Is the second stone black?


    @Override
    public void runOpMode() throws InterruptedException {


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
        frontLeft = getNewMotor("lf");
        frontRight = getNewMotor("rf");
        backLeft = getNewMotor("lb");
        backRight = getNewMotor("rb");

        //init accessory motors
        collectorLeft = getNewMotor("lla");
        collectorRight = getNewMotor("rla");
        linearSlide = getNewMotor("elevator");

        //init servos
        clamp = hardwareMap.crservo.get("clamp");
        foundation = hardwareMap.servo.get("foundation");
        rotation = hardwareMap.servo.get("rotation");
        release = hardwareMap.servo.get("release");



        if (frontLeft != null)
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (frontRight != null)
            frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (backLeft != null)
            backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (backRight != null)
            backRight.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        DriveForward(1.0, -5000); // Back up to stones
        DetectColorRGB(); // Determines the positions of the Skystone
        if (skystonePosition == 1) // Pattern A
        {
            MoveHook(1.0, 500); // Hook skystone
            DriveForward(1.0, 500); // Pull skystone out
            MoveHook(1, -500);// Raise hook out of the way
            //Collect block by moving forward while running collection device
            DriveForward(1.0, 5000);  //Move into a path for the alliance bridge
            TurnRight(1, 1500);
        } else if (skystonePosition == 2) // Pattern B
        {
            StrafeRight(1, -1000); // Align with Skystone
            MoveHook(1.0, 500); // Hook skystone
            DriveForward(1.0, 500); // Pull skystone out
            MoveHook(1, -500);// Raise hook out of the way
            //Collect block by moving forward while running collection device
            DriveForward(1.0, 5000);  //Move into a path for the alliance bridge
            TurnRight(1, 1500);
            DriveForward(1, 2500); //Meet up with other branches of code
        } else // Pattern C
        {
            StrafeRight(1, -2000); // Align with Skystone
            MoveHook(1.0, 500); // Hook skystone
            DriveForward(1.0, 500); // Pull skystone out
            MoveHook(1, -500);// Raise hook out of the way
            //Collect block by moving forward while running collection device
            DriveForward(1.0, 5000);  //Move into a path for the alliance bridge
            TurnRight(1, 1500);
            DriveForward(1, 5000); //Meet up with other branches of code
        }

        DriveForward(1, 7500); // Drive to  just before Foundation
        StrafeRight(1, 1500); // move right under foundation
        TurnRight(1, 3000); // Do a 180
        MoveHook(1, 500); // Hook foundation
        StrafeRight(1, 5000); // move foundation into building depot
        TurnRight(1, -3000); // Do a 180
        // PLace block in foundation
        DriveForward(1, 5000); // Park over tape

    }

    //Our software coach from last year helped us with this method that uses trigonometry to operate mecanum wheels
    public void DriveForward(double power, int distance) //Drive Forward
    {
        //resets encoder values
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sets Target position
        frontLeft.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);
        frontRight.setTargetPosition(distance);
        backRight.setTargetPosition(distance);

        //sets to runs to position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //runs
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        while (frontLeft.isBusy() && backLeft.isBusy() && frontRight.isBusy() && backRight.isBusy()) {                 //RED FLAG CHECK THIS THING!!!!
            //waits for all motors to stop
        }
        DF(0);  //sets power to 0

        //resets mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void MoveHook(doubleposition) //Drive Forward
    {

        foundation.setPosition(position);

    }

    public void DF(double power) {           //Method with no end, DO NOT USE UNLESS NECESSARY
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    public void TurnRight(double power, int distance)//Turns right  with power from 0-1 and for X encoders, turns left with a negative value
    {

        //resets encoder values
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sets Target position
        frontLeft.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);
        frontRight.setTargetPosition(-distance);
        backRight.setTargetPosition(-distance);

        //sets to runs to position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //runs
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
        while (frontLeft.isBusy() && backLeft.isBusy() && frontRight.isBusy() && backRight.isBusy()) {
            //waits for all motors to stop
        }
        DF(0);  //sets power to 0 to stop robot

        //resets mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //resets encoder values
//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        frontLeft.setPower(-power);
//        backLeft.setPower(-power);
//        frontRight.setPower(power);
//        backRight.setPower(power);
    }

    public void StrafeRight(double power, int distance) // strafes right with a power from 0-1, for X encoders, left if negative.
    {
        //resets encoder values
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sets Target position
        frontLeft.setTargetPosition(distance);
        backLeft.setTargetPosition(-distance);
        frontRight.setTargetPosition(-distance);
        backRight.setTargetPosition(distance);

        //sets to runs to position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //runs
        frontLeft.setPower(power);
        backLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(power);
        while (frontLeft.isBusy() && backLeft.isBusy() && frontRight.isBusy() && backRight.isBusy()) {
            //waits for all motors to stop
        }
        DF(0);  //sets power to 0 to stop robot

        //resets mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void DetectColorHSV(ColorSensor colorSensor, double SCALE_FACTOR, float hsvValues[]) {
        Color.RGBToHSV(
                (int) (blueColorSensor.red() * SCALE_FACTOR),
                (int) (blueColorSensor.green() * SCALE_FACTOR),
                (int) (blueColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        if (hsvValues[2] < 10) {
            telemetry.addData("Block: ", "Stone");
        } else {
            telemetry.addData("Block: ", "SkyStone");
        }

    }

    public void DetectColorRGB() {
        int blueSensorColorValueRed = blueColorSensor.red(); // red value from 0-255 from the blue color sensor
        int blueSensorColorValueBlue = blueColorSensor.blue(); // blue value from 0-255 from the blue color sensor
        int blueSensorColorValueGreen = blueColorSensor.green(); // green value from 0-255 from the blue color sensor

        int redSensorColorValueRed = redColorSensor.red(); // red value from 0-255 from the red color sensor
        int redSensorColorValueBlue = redColorSensor.blue(); // blue value from 0-255 from the red color sensor
        int redSensorColorValueGreen = redColorSensor.green(); // green value from 0-255 from the red color sensor


        if (blueSensorColorValueRed == 0 && blueSensorColorValueBlue == 0 && blueSensorColorValueGreen == 0) {
            skystonePosition = 1;
            telemetry.addData("Block ", "SkyStone");
            telemetry.addData("Pattern ", "A");
            telemetry.update();

        } else {
            Stone1isBlack = false;
            telemetry.addData("Block: ", "Stone");
            telemetry.update();

        }
        if (redSensorColorValueRed == 0 && redSensorColorValueBlue == 0 && redSensorColorValueGreen == 0) {
            skystonePosition = 2;
            telemetry.addData("Block: ", "SkyStone");
            telemetry.addData("Position ", "B");
            telemetry.update();
        } else {
            Stone2isBlack = false;
            telemetry.addData("Block: ", "Stone");

        }

        if (!Stone1isBlack && !Stone2isBlack) {
            skystonePosition = 3;
            telemetry.addData("Pattern ", "C");
            telemetry.update();
        }

    }
}

