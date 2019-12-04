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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;


/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

@TeleOp(name = "Tele-Op 2019 - 2020 Test 2", group = "Linear Opmode")
@Disabled
public class TeleOp20192020_Test_2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left = null;
    private DcMotor rear_left = null;
    private DcMotor front_right = null;
    private DcMotor rear_right = null;
    private DcMotor lift_left = null;
    private DcMotor lift_right = null;
    private DcMotor feeder_motor = null;
    private DcMotor top_motor = null;
    private Servo Clamp_Left = null;
    private Servo Clamp_Right = null;
    private Servo Feeder_Servo = null;
    private Servo Block_Pickup = null;
    private Servo End_Left = null;
    private Servo End_Right = null;
    private Servo Block_Kickout = null;
    private Servo Capstone = null;
    private Servo Release_Servo = null;
    private DigitalChannel Top_Sensor_Front = null;
    private DigitalChannel Top_Sensor_Rear = null;
    private DigitalChannel bottom_touch = null;
    private DigitalChannel top_touch = null;
    private int front_left_position; //variable to hold encoder position
    private int rear_left_position; //variable to hold encoder position
    private int front_right_position; //variable to hold encoder position
    private int rear_right_position; //variable to hold encoder position
    private int crane_state = 0;
    private float front_left_modifier = 0;
    private float rear_left_modifier = 0;
    private float front_right_modifier = 0;
    private float rear_right_modifier = 0;


    float feederServoPosition = 0;

    //@Disabled


    /*
     * Change the pattern every 10 seconds in AUTO mode.
     */
    private final static int LED_PERIOD = 10;

    /*
     * Rate limit gamepad button presses to every 500ms.
     */
    private final static int GAMEPAD_LOCKOUT = 500;

    com.qualcomm.hardware.rev.RevBlinkinLedDriver blinkinLedDriver;
    com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern pattern;

    Telemetry.Item patternName;
    Telemetry.Item display;
    org.firstinspires.ftc.teamcode.RevBlinkinLedDriver.DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    DigitalChannel blockbutton; // Hardware Device Object


    protected enum DisplayKind {
        MANUAL,
        AUTO
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        lift_left = hardwareMap.get(DcMotor.class, "lift_left");
        lift_right = hardwareMap.get(DcMotor.class, "lift_right");
        feeder_motor = hardwareMap.get(DcMotor.class, "feeder_motor");
        top_motor = hardwareMap.get(DcMotor.class, "top_motor");
        Clamp_Left = hardwareMap.get(Servo.class, "Clamp_Left");
        Clamp_Right = hardwareMap.get(Servo.class, "Clamp_Right");
        Feeder_Servo = hardwareMap.get(Servo.class, "Feeder_Servo");
        Block_Pickup = hardwareMap.get(Servo.class, "Block_Pickup");
        Capstone = hardwareMap.get(Servo.class, "Capstone");
        End_Left = hardwareMap.get(Servo.class, "End_Left");
        End_Right = hardwareMap.get(Servo.class, "End_Right");
        Block_Kickout = hardwareMap.get(Servo.class, "Block_Kickout");
        Release_Servo = hardwareMap.get(Servo.class, "Release_Servo");
        Top_Sensor_Rear = hardwareMap.get(DigitalChannel.class, "Top_Sensor_Rear");
        Top_Sensor_Front = hardwareMap.get(DigitalChannel.class, "Top_Sensor_Front");
        bottom_touch = hardwareMap.get(DigitalChannel.class, "bottom_touch");
        top_touch = hardwareMap.get(DigitalChannel.class, "top_touch");
        blockbutton = hardwareMap.get(DigitalChannel.class, "blockbutton");


        lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // set digital channel to input mode.
        Top_Sensor_Front.setMode(DigitalChannel.Mode.INPUT);
        Top_Sensor_Rear.setMode(DigitalChannel.Mode.INPUT);
        bottom_touch.setMode(DigitalChannel.Mode.INPUT);
        top_touch.setMode(DigitalChannel.Mode.INPUT);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        front_left.setDirection(DcMotor.Direction.FORWARD);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.FORWARD);
        lift_left.setDirection(DcMotor.Direction.REVERSE);
        lift_right.setDirection(DcMotor.Direction.REVERSE);
        feeder_motor.setDirection(DcMotor.Direction.REVERSE);
        top_motor.setDirection(DcMotor.Direction.FORWARD);

        displayKind = RevBlinkinLedDriver.DisplayKind.AUTO;

        blinkinLedDriver = hardwareMap.get(com.qualcomm.hardware.rev.RevBlinkinLedDriver.class, "blinkinLedDriver");
        pattern = com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.ORANGE;


        blinkinLedDriver.setPattern(pattern);

        display = telemetry.addData("Display Kind: ", displayKind.toString());
        patternName = telemetry.addData("Pattern: ", pattern.toString());

        // set the digital channel to input.
        blockbutton.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Single Cycle", "Incomplete");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (

                opModeIsActive()) {


            if (blockbutton.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
                //set color black
                pattern = com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BLACK;

            } else {
                telemetry.addData("Digital Touch", "Is Pressed");
                ///SET COLOR ORANGE
                pattern = com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.ORANGE;

            }
            blinkinLedDriver.setPattern(pattern);
            telemetry.update();

        /*
        if (displayKind == DisplayKind.AUTO) {

            doAutoDisplay();
        } else {

            // * MANUAL mode: Nothing to do, setting the pattern as a result of a gamepad event.

        }
        */


            //Player 1

            UpdateClamps();

            UpdateFeeder();

            UpdateDriveTrain();

            UpdateLift();

            UpdateCrane();

            UpdateBlockPickUp();

            UpdateCapstone();

            UpdateEndServo();

            //UpdateBlockKickout();

            UpdateReleaseServo();

            telemetry.addData("Single Cycle", "Complete");
            telemetry.update();
        }
    }


    public void UpdateClamps() {
        //Clamps
        if (gamepad1.left_bumper) {
            telemetry.addData("Clamps", "Clamp Up");
            Clamp_Left.setPosition(0f);
            Clamp_Right.setPosition(1f);

        } else if (gamepad1.left_trigger > 0) {
            telemetry.addData("Clamps", "Clamp Down");
            Clamp_Left.setPosition(0.8f);
            Clamp_Right.setPosition(0f);

        } else {
            telemetry.addData("Clamps", "Not Moving");
        }
    }


    public void UpdateFeeder() {

        //Feeder in Player 1
        if (gamepad1.x || gamepad2.left_trigger > 0) {
            telemetry.addData("Feeder", "feeder out");
            feeder_motor.setPower(1);

        } else if (gamepad1.a || gamepad2.right_stick_button) {
            telemetry.addData("Feeder", "Not moving");
            feeder_motor.setPower(0);

        } else if (gamepad1.y || gamepad2.right_trigger > 0) {
            telemetry.addData("Feeder", "Not moving");
            feeder_motor.setPower(-1);

        } else if (gamepad1.b) {
            telemetry.addData("Feeder", "Not moving");
            feeder_motor.setPower(0);

        } else {
            telemetry.addData("Feeder", "Not moving");
        }

    }


   /* public void UpdateFeederServo() {
        //feeder Servo Open Close
        if (gamepad1.right_trigger > 0 && feederServoPosition < 1) {
            telemetry.addData("FeederServo", "Feeder Servo Close");
            telemetry.addData("Angle", feederServoPosition);

            Feeder_Servo.setPosition(feederServoPosition);
            feederServoPosition = feederServoPosition + 0.1f;

        } else if (gamepad1.right_bumper && feederServoPosition > 0) {
            telemetry.addData("FeederServo", "Feeder Servo Open");
            telemetry.addData("Angle", feederServoPosition);
            Feeder_Servo.setPosition(feederServoPosition);
            feederServoPosition = feederServoPosition - 0.1f;

        }
    }

    */

    public void UpdateLift() {
        //Player 2

        //lift
        if (gamepad2.right_stick_y > 0 & bottom_touch.getState()) {
            telemetry.addData("Lift", "Lift up");
            lift_left.setPower(1);
            lift_right.setPower(1);

        } else if (gamepad2.right_stick_y < 0 && top_touch.getState()) {
            telemetry.addData("Lift", "Lift down");
            lift_left.setPower(-1);
            lift_right.setPower(-1);

        } else {
            telemetry.addData("Lift", "Not moving");
            lift_left.setPower(0);
            lift_right.setPower(0);
        }
    }


    public void UpdateCrane() {
//Crane
//            if (gamepad2.dpad_left && Top_Sensor_Rear.getState()) {
//                telemetry.addData("Crane", "Crane is moving forward");
//                top_motor.setPower(1);
//
//            } else if (gamepad2.dpad_right && Top_Sensor_Front.getState()) {
//                telemetry.addData("Crane", "Crane is moving backward");
//                top_motor.setPower(-1);
//
//            } else {
//                telemetry.addData("Crane", "Not Moving");
//                top_motor.setPower(0);
//            }
//
        if (gamepad2.dpad_left) {
            crane_state = 1;

        } else if (gamepad2.dpad_right) {
            crane_state = 2;
        } else if (gamepad2.dpad_down) {
            crane_state = 0;
        }


        if (crane_state == 0) {
            top_motor.setPower(0);

        } else if (crane_state == 1 && Top_Sensor_Rear.getState()) {
            top_motor.setPower(1);

        } else if (crane_state == 2 && Top_Sensor_Front.getState()) {
            top_motor.setPower(-1);

        } else {
            crane_state = 0;
            top_motor.setPower(0);
        }
    }

    public void UpdateDriveTrain() {

        telemetry.addData("front_left Encoder Position", front_left.getCurrentPosition());
        telemetry.addData("rear_right Encoder Position", rear_left.getCurrentPosition());
        telemetry.addData("front_left Encoder Position", front_right.getCurrentPosition());
        telemetry.addData("rear_right Encoder Position", rear_right.getCurrentPosition());

        double leftPower;
        double rightPower;
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        if (leftPower != 0 || rightPower != 0) {
            front_left.setPower(leftPower);
            rear_left.setPower(leftPower);
            front_right.setPower(rightPower);
            rear_right.setPower(rightPower);
        } else if (gamepad1.right_stick_x < -0.4 && gamepad1.right_stick_y < 0) {
            telemetry.addData("DriveTrain", "Up and Left");
            front_left.setPower((1 + front_left_modifier));
            rear_left.setPower(-(1 + rear_left_modifier));
            front_right.setPower(-(1 + front_right_modifier));
            rear_right.setPower((1 + rear_right_modifier));

           /* front_left.setPower(-(0.5 + front_left_modifier));
            rear_left.setPower((1 + rear_left_modifier));
            front_right.setPower((1 + front_right_modifier));
            rear_right.setPower(-(0.5 + rear_right_modifier));

            */

        } else if (gamepad1.right_stick_x > 0.4 && gamepad1.right_stick_y < 0) {
            telemetry.addData("DriveTrain", "Up and Right");
            front_left.setPower(-(1 + front_left_modifier));
            rear_left.setPower((1 + rear_left_modifier));
            front_right.setPower((1 + front_right_modifier));
            rear_right.setPower(-(1 + rear_right_modifier));

            /*front_left.setPower((1 + front_left_modifier));
            rear_left.setPower(-(.5 + rear_left_modifier));
            front_right.setPower(-(.5 + front_right_modifier));
            rear_right.setPower((1 + rear_right_modifier));

             */

        } else if (gamepad1.right_stick_x < -0.4 && gamepad1.right_stick_y > 0) {
            telemetry.addData("DriveTrain", "Down and Left");
            front_left.setPower((1 + front_left_modifier));
            rear_left.setPower(-(1 + rear_left_modifier));
            front_right.setPower(-(1 + front_right_modifier));
            rear_right.setPower((1 + rear_right_modifier));

            /*front_left.setPower(-(1 + front_left_modifier));
            rear_left.setPower((0.5 + rear_left_modifier));
            front_right.setPower((0.5 + front_right_modifier));
            rear_right.setPower(-(1 + rear_right_modifier));

             */

        } else if (gamepad1.right_stick_x > 0.4 && gamepad1.right_stick_y > 0) {
            telemetry.addData("DriveTrain", "Down and Right");
            front_left.setPower(-(1 + front_left_modifier));
            rear_left.setPower((1 + rear_left_modifier));
            front_right.setPower((1 + front_right_modifier));
            rear_right.setPower(-(1 + rear_right_modifier));

           /* front_left.setPower((0.5 + front_left_modifier));
            rear_left.setPower(-(1 + rear_left_modifier));
            front_right.setPower(-(1 + front_right_modifier));
            rear_right.setPower((0.5 + rear_right_modifier));

            */

        } else if (gamepad1.right_stick_y > 0) {
            telemetry.addData("DriveTrain", "Moving Backwards");
            front_left.setPower(-(1 + front_left_modifier));
            rear_left.setPower(-(1 + rear_left_modifier));
            front_right.setPower(-(1 + front_right_modifier));
            rear_right.setPower(-(1 + rear_right_modifier));

        } else if (gamepad1.right_stick_y < 0) {
            telemetry.addData("DriveTrain", "Moving Forward");

            front_left.setPower((1 + front_left_modifier));
            rear_left.setPower((1 + rear_left_modifier));
            front_right.setPower((1 + front_right_modifier));
            rear_right.setPower((1 + rear_right_modifier));

        } else if (gamepad1.right_stick_x > 0) {
            telemetry.addData("DriveTrain", "Strafing Right");
            front_left.setPower(-(1 + front_left_modifier));
            rear_left.setPower((1 + rear_left_modifier));
            front_right.setPower((1 + front_right_modifier));
            rear_right.setPower(-(1 + rear_right_modifier));

        } else if (gamepad1.right_stick_x < 0) {
            telemetry.addData("DriveTrain", "Strafing Left");
            front_left.setPower((1 + front_left_modifier));
            rear_left.setPower(-(1 + rear_left_modifier));
            front_right.setPower(-(1 + front_right_modifier));
            rear_right.setPower((1 + rear_right_modifier));

        } else if (gamepad1.dpad_right) {
            telemetry.addData("DriveTrain", "Strafing Right");
            front_left.setPower(-(1 + front_left_modifier));
            rear_left.setPower((1 + rear_left_modifier));
            front_right.setPower((1 + front_right_modifier));
            rear_right.setPower(-(1 + rear_right_modifier));

        } else if (gamepad1.dpad_left) {
            telemetry.addData("DriveTrain", "Strafing Left");
            front_left.setPower((1 + front_left_modifier));
            rear_left.setPower(-(1 + rear_left_modifier));
            front_right.setPower(-(1 + front_right_modifier));
            rear_right.setPower((1 + rear_right_modifier));

        } else {
            telemetry.addData("DriveTrain", "Not moving");
            front_left.setPower(0);
            rear_left.setPower(0);
            front_right.setPower(0);
            rear_right.setPower(0);
        }
    }

    public void UpdateBlockPickUp() {
        //Block pick up
        if (gamepad2.right_bumper) {
            telemetry.addData("BlockPickUp", "Block pickup open ");
            Block_Pickup.setPosition(0.4f);

        } else if (gamepad2.left_bumper) {
            telemetry.addData("BlockPickUp", "Block pickup closed");
            Block_Pickup.setPosition(1f);

        } else {
            telemetry.addData("BlockPickUp", "Not moving");
        }
    }

    public void UpdateCapstone() {
        //cap stone
        if (gamepad2.a) {
            telemetry.addData("Capstone", "Capstone");
            Capstone.setPosition(0);
        }
    }

    public void UpdateEndServo() {
        //end servo
        if (gamepad2.x) {
            telemetry.addData("EndServo", "end servos");
            End_Left.setPosition(.8);
            End_Right.setPosition(0);
        }
    }

    public void UpdateReleaseServo() {
        //release servo
        if (gamepad2.y) {
            telemetry.addData("ReleaseServo", "feeder release");
            Release_Servo.setPosition(0.2);
        }
    }

    protected void handleGamepad() {
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad1.a) {
            setDisplayKind(RevBlinkinLedDriver.DisplayKind.MANUAL);
            gamepadRateLimit.reset();
        } else if (gamepad1.b) {
            setDisplayKind(RevBlinkinLedDriver.DisplayKind.AUTO);
            gamepadRateLimit.reset();
        } else if ((displayKind == RevBlinkinLedDriver.DisplayKind.MANUAL) && (gamepad1.left_bumper)) {
            pattern = pattern.previous();
            displayPattern();
            gamepadRateLimit.reset();
        } else if ((displayKind == RevBlinkinLedDriver.DisplayKind.MANUAL) && (gamepad1.right_bumper)) {
            pattern = pattern.next();
            displayPattern();
            gamepadRateLimit.reset();

        }
    }

    protected void setDisplayKind(RevBlinkinLedDriver.DisplayKind displayKind) {
        this.displayKind = displayKind;
        display.setValue(displayKind.toString());
    }

    protected void doAutoDisplay() {
        if (ledCycleDeadline.hasExpired()) {
            pattern = pattern.next();
            displayPattern();
            ledCycleDeadline.reset();
        }
    }

    protected void displayPattern() {
        blinkinLedDriver.setPattern(pattern);
        patternName.setValue(pattern.toString());
    }
}

