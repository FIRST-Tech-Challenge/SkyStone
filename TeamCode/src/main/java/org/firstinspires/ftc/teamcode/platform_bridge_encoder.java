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
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DigitalChannel;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

@Autonomous(name = "platform_bridge_encoder   ", group = "Linear Opmode")
@Disabled
public class platform_bridge_encoder extends LinearOpMode {

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
    private int rear_left_position; //variable to hold encoder position
    private int front_left_position; //variable to hold encoder position
    private int front_right_position; //variable to hold encoder position
    private int rear_right_position; //variable to hold encoder position
    private int crane_state = 0;
    private float front_left_modifier = 0;
    private float rear_left_modifier = 0;
    private float front_right_modifier = 0;
    private float rear_right_modifier = 0;


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
        End_Left = hardwareMap.get(Servo.class, "End_Left");
        End_Right = hardwareMap.get(Servo.class, "End_Right");
        Block_Kickout = hardwareMap.get(Servo.class, "Block_Kickout");
        Release_Servo = hardwareMap.get(Servo.class, "Release_Servo");
        Top_Sensor_Rear = hardwareMap.get(DigitalChannel.class, "Top_Sensor_Rear");
        Top_Sensor_Front = hardwareMap.get(DigitalChannel.class, "Top_Sensor_Front");
        bottom_touch = hardwareMap.get(DigitalChannel.class, "bottom_touch");
        top_touch = hardwareMap.get(DigitalChannel.class, "top_touch");


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

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        Clamp_Left.setPosition(0);
        Clamp_Right.setPosition(.85);
        boolean Stage1Complete = false;
        boolean Stage2 = false;
        boolean Stage3 = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        sleep(100);

        while (opModeIsActive()) {

            if (front_left.getCurrentPosition() <= 1921 && Stage1Complete == false) {

                front_left.setPower(.8);
                front_right.setPower(.8);
                rear_left.setPower(.8);
                rear_right.setPower(.8);


            } else {

                Clamp_Left.setPosition(0.85f);
                Clamp_Right.setPosition(0.15f);
                Stage1Complete = true;
                Stage2 = true;

            }

            if (front_left.getCurrentPosition() >= 500 && Stage2 == true) {

                front_left.setPower(.8);
                front_right.setPower(.8);
                rear_left.setPower(.8);
                rear_right.setPower(.8);

            } else {

                front_left.setPower(0);
                front_right.setPower(0);
                rear_left.setPower(0);
                rear_right.setPower(0);
                Stage3 = true;

            }


        }
        //clamps go down

        /*Clamp_Left.setPosition(0.85f);
        Clamp_Right.setPosition(0.15f);
        sleep(500);

        //robot moves backwards

        front_left.setPower(-.8);
        front_right.setPower(-.8);
        rear_left.setPower(-.8);
        rear_right.setPower(-.8);
        sleep(800);

        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);

        //strafes left

        front_left.setPower(.8);
        front_right.setPower(-.8);
        rear_left.setPower(-.8);
        rear_right.setPower(.8);
        sleep(1000);

        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);

        //robot moves forward

        front_left.setPower(.8);
        front_right.setPower(.8);
        rear_left.setPower(.8);
        rear_right.setPower(.8);
        sleep(800);

        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);

        //clamps go up

        Clamp_Left.setPosition(0);
        Clamp_Right.setPosition(.85);

        //robot moves back

        front_left.setPower(.8);
        front_right.setPower(.8);
        rear_left.setPower(.8);
        rear_right.setPower(.8);
        sleep(2000);

        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);
        */


    }
}

