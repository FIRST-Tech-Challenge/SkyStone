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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

@TeleOp(name = "BC Nelms jm", group = "Linear Opmode")
public class BA_Nelms_jm extends LinearOpMode {

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
    private Servo Capstone = null;
    private DigitalChannel Top_Sensor_Front = null;
    private DigitalChannel Top_Sensor_Rear = null;
    private DigitalChannel bottom_touch = null;
    private DigitalChannel top_touch = null;




    float feederServoPosition = 0;


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
        Top_Sensor_Rear = hardwareMap.get(DigitalChannel.class, "Top_Sensor_Rear");
        Top_Sensor_Front = hardwareMap.get(DigitalChannel.class, "Top_Sensor_Front");
        bottom_touch = hardwareMap.get(DigitalChannel.class, "bottom_touch");
        top_touch = hardwareMap.get(DigitalChannel.class, "top_touch");


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


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            //Player 1

            //Strafing
            if (gamepad1.right_stick_x < -0.2 && gamepad1.right_stick_y < 0) {
                telemetry.addData("Status", "Up and Left");
                telemetry.update();
                front_left.setPower(-0.5);
                rear_left.setPower(1);
                front_right.setPower(1);
                rear_right.setPower(-0.5);

            } else if (gamepad1.right_stick_x > 0.2 && gamepad1.right_stick_y < 0) {
                telemetry.addData("Status", "Up and Right");
                telemetry.update();
                front_left.setPower(1);
                rear_left.setPower(-0.5);
                front_right.setPower(-0.5);
                rear_right.setPower(1);

            } else if (gamepad1.right_stick_x < -0.2 && gamepad1.right_stick_y > 0) {
                telemetry.addData("Status", "Down and Left");
                telemetry.update();
                front_left.setPower(-1);
                rear_left.setPower(0.5);
                front_right.setPower(0.5);
                rear_right.setPower(-1);

            } else if (gamepad1.right_stick_x > 0.2 && gamepad1.right_stick_y > 0) {
                telemetry.addData("Status", "Down and Right");
                telemetry.update();
                front_left.setPower(0.5);
                rear_left.setPower(-1);
                front_right.setPower(-1);
                rear_right.setPower(0.5);

            } else if (gamepad1.right_stick_y > 0) {
                telemetry.addData("Status", "Moving Backwards");
                telemetry.update();
                front_left.setPower(-1);
                rear_left.setPower(-1);
                front_right.setPower(-1);
                rear_right.setPower(-1);

            } else if (gamepad1.right_stick_y < 0) {
                telemetry.addData("Status", "Moving Forward");
                telemetry.update();
                front_left.setPower(1);
                rear_left.setPower(1);
                front_right.setPower(1);
                rear_right.setPower(1);

            } else if (gamepad1.right_stick_x > 0) {
                telemetry.addData("Status", "Strafing Right");
                telemetry.update();
                front_left.setPower(1);
                rear_left.setPower(-1);
                front_right.setPower(-1);
                rear_right.setPower(1);

            } else if (gamepad1.right_stick_x < 0) {
                telemetry.addData("Status", "Strafing Left");
                telemetry.update();
                front_left.setPower(-1);
                rear_left.setPower(1);
                front_right.setPower(1);
                rear_right.setPower(-1);

            } else if (gamepad1.dpad_right) {
                telemetry.addData("Status", "Turning Right");
                telemetry.update();
                front_left.setPower(1);
                rear_left.setPower(1);
                front_right.setPower(-1);
                rear_right.setPower(-1);

            } else if (gamepad1.dpad_left) {
                telemetry.addData("Status", "Turning Left");
                telemetry.update();
                front_left.setPower(-1);
                rear_left.setPower(-1);
                front_right.setPower(1);
                rear_right.setPower(1);
            } else {
                telemetry.addData("Status", "Not moving");
                telemetry.update();
                front_left.setPower(0);
                rear_left.setPower(0);
                front_right.setPower(0);
                rear_right.setPower(0);
            }


            //Clamps
            if (gamepad1.left_bumper) {
                telemetry.addData("Status", "Clamp Down");
                telemetry.update();
                Clamp_Left.setPosition(0.83f);
                Clamp_Right.setPosition(0.17f);
            }

            else if (gamepad1.left_trigger > 0) {
                    telemetry.addData("Status", "Clamp Up");
                    telemetry.update();
                    Clamp_Left.setPosition(0.4f);
                    Clamp_Right.setPosition(0.6f);

            }
            else{
                telemetry.addData("Status", "Not Moving");
                telemetry.update();
            }


            //Feeder in
            if (gamepad1.y) {
                telemetry.addData("Status", "feeder in on");
                telemetry.update();
                feeder_motor.setPower(1);

            } else if (gamepad1.b) {
                telemetry.addData("Status", "Not moving");
                telemetry.update();
                feeder_motor.setPower(0);

            } else {

                telemetry.addData("Status", "Not moving");
                telemetry.update();
            }

            //Feeder out
            if (gamepad1.x) {
                telemetry.addData("Status", "Not moving");
                telemetry.update();
                feeder_motor.setPower(-1);

            } else if (gamepad1.a) {
                telemetry.addData("Status", "Not moving");
                telemetry.update();
                feeder_motor.setPower(0);

            } else {
                telemetry.addData("Status", "Not moving");
                telemetry.update();
            }


            //feeder Servo Open Close
            if (gamepad1.right_trigger > 0 && feederServoPosition < 1) {
                telemetry.addData("Status", "Feeder Servo Close");
                telemetry.addData("Angle", feederServoPosition);
                telemetry.update();
                Feeder_Servo.setPosition(feederServoPosition);
                feederServoPosition = feederServoPosition + 0.001f;

            } else if (gamepad1.right_bumper && feederServoPosition > 0) {
                telemetry.addData("Status", "Feeder Servo Open");
                telemetry.addData("Angle", feederServoPosition);
                telemetry.update();
                Feeder_Servo.setPosition(feederServoPosition);
                feederServoPosition = feederServoPosition - 0.001f;

            }


            //Player 2

            //lift
            if (gamepad2.right_stick_y > 0) {
                telemetry.addData("Status", "Lift up");
                telemetry.update();
                lift_left.setPower(1);
                lift_right.setPower(1);

            } else if (gamepad2.right_stick_y < 0) {
                telemetry.addData("Status", "Lift down");
                telemetry.update();
                lift_left.setPower(-1);
                lift_right.setPower(-1);

            } else {
                telemetry.addData("Status", "Not moving");
                telemetry.update();
                lift_left.setPower(0);
                lift_right.setPower(0);
            }


            //Crain
            if (gamepad2.dpad_left && Top_Sensor_Rear.getState()) {
                telemetry.addData("Status", "Crain is moving foward");
                top_motor.setPower(1);

            } else if (gamepad2.dpad_right && Top_Sensor_Front.getState()) {
                telemetry.addData("Status", "Crain is moving backward");
                top_motor.setPower(-1);

            } else {
                telemetry.addData("Status", "Not Moving");
                top_motor.setPower(0);
            }


            //Block pick up
            if (gamepad2.right_bumper) {
                telemetry.addData("Status", "Block pickup open ");
                telemetry.update();
                Block_Pickup.setPosition(0f);

            } else if (gamepad2.left_bumper) {
                telemetry.addData("Status", "Block pickup closed");
                telemetry.update();
                Block_Pickup.setPosition(0.5f);

            } else {
                telemetry.addData("Status", "Not moving");
                telemetry.update();
            }
        }

    }
}


