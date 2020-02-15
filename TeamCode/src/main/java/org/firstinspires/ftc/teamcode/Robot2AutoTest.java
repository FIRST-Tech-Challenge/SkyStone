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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import java.lang.reflect.Array;
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

@Autonomous(name="Robot2AutoTest", group="Linear OpMode")
//@Disabled
public class Robot2AutoTest extends LinearOpMode {


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
    private Servo clamp = null;
    private Servo rotation = null;
    private Servo foundationLeft = null;
    private Servo foundationRight = null;
    private CRServo release = null;
    private Servo capstone = null;
    private Servo sideArm = null;
    private CRServo extenderTop = null;
    private CRServo extenderBottom = null;

    //Sensors
    private ColorSensor blueColorSensor = null;
    private ColorSensor redColorSensor = null;

    // Other
    int skystonePosition; // Can equal 1, 2, or 3. This corresponds to the A, B and C patterns.
    boolean Stone1isYellow; // Is the first stone black?
    boolean Stone2isYellow; // Is the second stone black?

    int colorDiff;

    // int red = new int[]{10,10,10};  //(red, green, blue) values for yellow currently, this is the color to be searched for when detecting

    final int backward = 1;
    final int forward = -1;
    final int left = -1;
    final int right = 1;
    final int encoderBack = -1;
    final int encoderForward = 1;





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
        clamp = hardwareMap.servo.get("clamp");
        rotation = hardwareMap.servo.get("rotation");
        release = hardwareMap.crservo.get("release");
        foundationLeft = hardwareMap.servo.get("foundationLeft");
        foundationRight = hardwareMap.servo.get("foundationRight");
        extenderTop = hardwareMap.crservo.get("extenderTop");
        extenderBottom = hardwareMap.crservo.get("extenderBottom");
        sideArm = hardwareMap.servo.get("sideArm");


        //init sensors
        blueColorSensor = hardwareMap.colorSensor.get("blueColorSensor");
        redColorSensor = hardwareMap.colorSensor.get("redColorSensor");


        if (frontLeft != null)
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
        if (frontRight != null)
            frontRight.setDirection(DcMotor.Direction.REVERSE);
        if (backLeft != null)
            backLeft.setDirection(DcMotor.Direction.FORWARD);
        if (backRight != null)
            backRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Back Left Encoder Value: ", backLeft.getCurrentPosition());
        telemetry.update();




        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //ReleaseCollector(-1.0);
        //AutoMecanumMove(1500*EncoderBack, 0, 0.5, -0.1);
        //Strafe DiagonalLeftForward
        AutoMecanumMove(1500 * encoderForward, 0.0 * right, 0.5 * forward, 0.025); //Drive Forward
        AutoMecanumMove(1500 * encoderForward, 0.5 * left, 0.0 * forward, 0.025); //Strafe Left
        ReleaseCollector(1.0);
        CollectorGo(1.0,1.0);
        sleep(500);
        CollectorStop();
        MoveHooks(0.27, 0.37); //Perpendicular
        MoveHooks(0.27, 0.67); //Parallel
        MoveSideArm(0.0); //Up
        MoveSideArm(0.5); //Down
        DetectColorRGB();



    }
    private void AutoMecanumMove(int targetVal, double leftStickX, double leftStickY, double rightStickX)
    {
        ResetEncoder();


        if(targetVal < 0)
            while (frontLeft.getCurrentPosition() >= targetVal && backLeft.getCurrentPosition() >= targetVal && frontRight.getCurrentPosition() >= targetVal && backRight.getCurrentPosition() >= targetVal)
            {
                mecanumMove(leftStickX, leftStickY, rightStickX); //(-0.5, 0.5, -0.02 DiagonalLeft),(-0.5, 0, 0 StrafeLeft), (0.5, 0, 0 StrafeRight), (0, -0.5, 0 Forward)

                telemetry.addData("Back Left Encoder Value: ", backLeft.getCurrentPosition());
                telemetry.update();

            }
        if(targetVal >=0 )
            while (frontLeft.getCurrentPosition() <= targetVal && backLeft.getCurrentPosition() <= targetVal && frontRight.getCurrentPosition() <= targetVal && backRight.getCurrentPosition() <= targetVal)
            {
                mecanumMove(leftStickX, leftStickY, rightStickX); //(-0.5, 0.5, -0.02 DiagonalLeft),

                telemetry.addData("Back Left Encoder Value: ", backLeft.getCurrentPosition());
                telemetry.update();

            }
        mecanumMove(0, 0, 0);
    }
    private void ResetEncoder(){

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("" + frontLeft.getMode());
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);


        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("" + frontLeft.getMode());

    }


    //Our software coach from last year helped us with this method that uses trigonometry to operate mecanum wheels
    private void mecanumMove(double leftStickX, double leftStickY, double rightStickX) {


        double distanceFromCenter = Math.sqrt(leftStickY * leftStickY + leftStickX * leftStickX);  // might be leftStickY * leftStickX This double uses the pythagorean theorem to find  out the distance from the the joystick center

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
    private void ReleaseCollector(double position) {

        release.setPower(position);

        sleep(500);

    }
    public void DetectColorRGB() {
        int leftSensorColorValueRed = blueColorSensor.red(); // red value from 0-255 from the blue color sensor
        // int leftSensorColorValueBlue = blueColorSensor.blue(); // blue value from 0-255 from the blue color sensor
        // int leftSensorColorValueGreen = blueColorSensor.green(); // green value from 0-255 from the blue color sensor

        int rightSensorColorValueRed = redColorSensor.red(); // red value from 0-255 from the red color sensor
        // int rightSensorColorValueBlue = redColorSensor.blue(); // blue value from 0-255 from the red color sensor
        // int rightSensorColorValueGreen = redColorSensor.green(); // green value from 0-255 from the red color sensor
        colorDiff = Math.abs(leftSensorColorValueRed - rightSensorColorValueRed);

        //if (colorDiff <= 5) skystonePosition = 3 Pattern C
        //if (colorDiff >= 5 && rightSensorColorValueRed < leftSensorColorValueRed) skystonePosition = 2 Pattern B
        //if (colorDiff >= 5 && rightSensorColorValueRed > leftSensorColorValueRed) skystonePosition = 1 Pattern A


        if (colorDiff > 5 && rightSensorColorValueRed > leftSensorColorValueRed) {

            Stone1isYellow = false;
            skystonePosition = 1;
            telemetry.addData("Block 1 is: ", "SkyStone");
            telemetry.addData("Pattern ", "A");
            telemetry.update();



        } else {

            Stone1isYellow = true;
            telemetry.addData("Block 1 is: ", "Stone");
            telemetry.update();


        }
        if (colorDiff > 5 && rightSensorColorValueRed < leftSensorColorValueRed) {

            Stone2isYellow = false;
            skystonePosition = 2;
            telemetry.addData("Block 2 is : ", "SkyStone");
            telemetry.addData("Pattern ", "B");
            telemetry.update();


        } else {

            Stone2isYellow = true;
            telemetry.addData("Block 2 is : ", "Stone");
            telemetry.update();

        }

        if (colorDiff < 5 || Stone1isYellow && Stone2isYellow) {
            skystonePosition = 3;
            telemetry.addData("Block 3 is : ", "SkyStone");
            telemetry.addData("Pattern ", "C");
            telemetry.update();
        }

    }

    private void CollectorWait(double leftPower, double rightPower, long time) //Wait for collectors to stop
    {

        collectorLeft.setPower(leftPower);
        collectorRight.setPower(rightPower);

        sleep(time);

        collectorLeft.setPower(0);
        collectorRight.setPower(0);

    }
    private void CollectorGo(double leftPower, double rightPower) //Continue with program
    {

        collectorLeft.setPower(leftPower);
        collectorRight.setPower(rightPower);

    }
    private void CollectorStop() //Stop Collectors
    {

        collectorLeft.setPower(0);
        collectorRight.setPower(0);

    }
    private void MoveHooks(double leftPosition, double rightPosition) {

        foundationLeft.setPosition(leftPosition);  //0.27 Perpendicular, 0.67 Parallel
        foundationRight.setPosition(right); //0.37 Perpendicular, 0.77 Parallel

        double foundationLeftPosition = foundationLeft.getPosition();
        double foundationRightPosition = foundationRight.getPosition();

        while(foundationLeftPosition != left && foundationRightPosition != rightPosition)
        {

            foundationLeftPosition = foundationLeft.getPosition();
            foundationRightPosition = foundationRight.getPosition();

        }
        sleep(500);

    }
    private void ExtendTape(double power, long time) {

        extenderTop.setPower(power);
        extenderBottom.setPower(power);

        sleep(time);

        extenderTop.setPower(0);
        extenderBottom.setPower(0);

    }

    public void MoveSideArm(double position) //Drive Forward
    {

        sideArm.setPosition(position); //UP 0.0, Down 0.5
        double sideArmPosition = sideArm.getPosition();
        while(sideArmPosition != position)
        {

            sideArmPosition = sideArm.getPosition();

        }
        sleep(500);


    }

}

