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
  */

 package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.util.ElapsedTime;


 /**
  * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
  * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
  * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
  * class is instantiated on the Robot Controller and executed.
  * <p>
  * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
  * It includes all the skeletal structure that all linear OpModes contain.
  * <p>
  * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
  * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
  */

 @Autonomous(name = "Unfold_Test4", group = "Linear Opmode")
//@Disabled
 public class Unfold_Test4 extends LinearOpMode {

     // Declare OpMode members.
     private ElapsedTime runtime = new ElapsedTime();
     private DcMotor front_left = null;
     private DcMotor rear_left = null;
     private DcMotor front_right = null;
     private DcMotor rear_right = null;
     private DcMotor lift_left = null;
     private DcMotor lift_right = null;
     private Servo Clamp_Right = null;
     private Servo Clamp_Left = null;
     private Servo Release_Servo = null;
     //private int front_left_position; //Variables to hold encoder position
     //private int front_right_position;
     //private int rear_left_position;
     //private int rear_right_position;
     //private int lift_left_position;
     //private int lift_right_position;

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
         Clamp_Right = hardwareMap.get(Servo.class, "Clamp_Right");
         Clamp_Left = hardwareMap.get(Servo.class, "Clamp_Left");
         Release_Servo = hardwareMap.get(Servo.class, "Release_Servo");

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
         front_right.setDirection(DcMotor.Direction.FORWARD);
         rear_right.setDirection(DcMotor.Direction.REVERSE);
         lift_left.setDirection(DcMotor.Direction.FORWARD);
         lift_right.setDirection(DcMotor.Direction.FORWARD);

         // Wait for the game to start (driver presses PLAY)
         waitForStart();
         runtime.reset();

         front_left.setPower(0);
         rear_left.setPower(0);
         front_right.setPower(0);
         rear_right.setPower(0);
         lift_left.setPower(0);
         lift_right.setPower(0);
         Clamp_Right.setPosition(.6f);
         Clamp_Left.setPosition(.5f);
         Release_Servo.setPosition(0);

         // run until the end of the match (driver presses STOP)

         //This code get the robot out of starting position


         goForward();
         while (opModeIsActive() && lift_left.getCurrentPosition() > -200) {

             liftDown();


         }
         liftStop();
         Clamp_Right.setPosition(.47f);
         Clamp_Left.setPosition(.53f);
         sleep(300);
         liftUp();

         if (lift_left.getCurrentPosition() > 200) {

             Release_Servo.setPosition(.5);

         }
         liftStop();


         telemetry.addData("Status", "Run Time: " + runtime.toString());
         telemetry.update();

     }

     public void goForward() {


         front_left.setPower(.7);
         rear_left.setPower(.7);
         front_right.setPower(.7);
         rear_right.setPower(.7);

         sleep(400);

         front_left.setPower(0);
         rear_left.setPower(0);
         front_right.setPower(0);
         rear_right.setPower(0);

     }

     public void liftDown() {
         lift_left.setPower(-1);
         lift_right.setPower(-1);
         telemetry.addData("Encoder liftDown() test", lift_left.getCurrentPosition());
         telemetry.update();
     }

     public void liftUp() {

         lift_left.setPower(1);
         lift_right.setPower(1);
         telemetry.addData("Encoder liftDown() test", lift_left.getCurrentPosition());
         telemetry.update();
     }

     public void liftStop() {

         lift_left.setPower(0);
         lift_right.setPower(0);
         telemetry.addData("Encoder liftDown() test", lift_left.getCurrentPosition());
         telemetry.update();
     }
 }
