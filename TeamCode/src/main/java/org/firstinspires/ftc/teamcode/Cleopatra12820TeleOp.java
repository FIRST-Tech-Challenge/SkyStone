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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 Tinoco's starting working code, I named the robot Cleopatra.
 */

@TeleOp(name="Basic: Linear OpMode for 1019-2020 Season", group="Linear Opmode")
//@Disabled
public class Cleopatra12820TeleOp extends LinearOpMode {
    double clawOffset = 0;                       // Servo mid position
    double servoOffset = 0;
    public static final double CLAW_SPEED = 0.02; // sets rate to move claw servo
    public static final double SERV_SPEED = 0.02; // sets rate to move claw servo
    // public static final double INCREMENT=0.01;     // amount to slew servo each CYCLE_MS cycle
    public static final int CYCLE_MS = 50;     // period of each cycle
    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER    =  0.45 ;
     public static final double ARM_DOWN_POWER  = -0.45 ;
     public static final double ELBOW_UP_POWER    =  0.45 ;
     public static final double ELBOW_DOWN_POWER  = -0.45 ;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double position = MID_SERVO;// Start at halfway position
    DcMotor intakeMotorRight = null;
    DcMotor intakeMotorLeft = null;
    DcMotor backMotorLeft = null;
    DcMotor backMotorRight = null;
    DcMotor frontMotorLeft = null;
    DcMotor frontMotorRight = null;
    DcMotor armElbow = null;
    DcMotor armWrist = null;
    //Servo intakeServoRight = null;
    //Servo intakeServoLeft = null;
    Servo claw = null;
    Servo rotator = null;


    public void setIntakeServoLeft(Servo intakeServoLeft) {
        intakeServoLeft.setPosition(MID_SERVO);
    }

    public void setIntakeServoRightt(Servo intakeServoRight) {
        intakeServoRight.setPosition(MID_SERVO);
    }

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();



        /*  Initialize the hardware variables. Note that the strings used here as parameters
            to 'get' must correspond to the names assigned during the robot configuration
            step (using the FTC Robot Controller app on the phone).*/


        intakeMotorLeft = hardwareMap.get(DcMotor.class, "intakeMotorLeft");
        intakeMotorRight = hardwareMap.get(DcMotor.class, "intakeMotorRight");
        backMotorLeft = hardwareMap.get(DcMotor.class, "backMotorLeft");
        backMotorRight = hardwareMap.get(DcMotor.class, "backMotorRight");
        frontMotorLeft = hardwareMap.get(DcMotor.class, "frontMotorLeft");
        frontMotorRight = hardwareMap.get(DcMotor.class, "frontMotorRight");
        armElbow = hardwareMap.get(DcMotor.class, "armElbow");
        armWrist = hardwareMap.get(DcMotor.class, "armWrist");
        //intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        //intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        claw = hardwareMap.get(Servo.class, "claw");
        rotator = hardwareMap.get(Servo.class, "rotator");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        armElbow.setDirection(DcMotor.Direction.FORWARD);
        armWrist.setDirection(DcMotor.Direction.REVERSE);
        intakeMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeMotorRight.setDirection(DcMotor.Direction.FORWARD);
        backMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        backMotorRight.setDirection(DcMotor.Direction.REVERSE);
        frontMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        frontMotorRight.setDirection(DcMotor.Direction.FORWARD);   //THIS IS THE ONE I CHANGED
        //intakeServoLeft.setDirection(Servo.Direction.REVERSE);
        //intakeServoRight.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);
        rotator.setDirection(Servo.Direction.FORWARD);
        //intakeServoRight.setPosition(0);
        //intakeServoLeft.setPosition(0);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //setIntakeServoLeft(intakeServoLeft);
        //setIntakeServoRightt(intakeServoRight);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double intakeLeftPower;
            double intakeRightPower;
            double Speed = -gamepad1.left_stick_y;
            double Turn = gamepad1.left_stick_x;
            double Strafe = gamepad1.right_stick_x;
            //double MAX_SPEED = 1.0;

            //telemetry.addData("Right Servo position", intakeServoRight.getPosition());
            //telemetry.update();

/*
      Front Left= +Speed + Turn - Strafe      Front Right= +Speed - Turn + Strafe
      Back Left  = +Speed + Turn + Strafe      Back Right = +Speed - Turn - Strafe
*/
            double numFl = Range.clip((+Speed + Turn - Strafe), -1, +1);
            double numBl = Range.clip((+Speed + Turn + Strafe), -1, 1);
            double numFr = Range.clip((+Speed - Turn + Strafe), -1, +1);
            double numBr = Range.clip((+Speed - Turn - Strafe), -1, 1);
            double armElbowPower = Range.clip(0.5*gamepad2.left_stick_y, -1, 1);
            double armWristPower = Range.clip(2 *gamepad2.right_stick_y, -1, 1);

            armWrist.setPower(armWristPower);
            armElbow.setPower(armElbowPower);



            /*Sets the servo positions to their "home" position when the Teleop stage begins
            intakeServoRight.setPosition(0.5);
            intakeServoLeft.setPosition(0.5);
            claw.setPosition(0.5);
            rotator.setPosition(0.5);*/

            if (gamepad2.left_trigger > 0) {
                intakeLeftPower = Range.clip(gamepad2.left_trigger, -1.0, 1.0); //swaps between the two programed "extremes" on the servo
                intakeRightPower = Range.clip(gamepad2.left_trigger, -1.0, 1.0);
            } else {
                intakeLeftPower = Range.clip(-gamepad2.right_trigger, -1.0, 1.0);
                intakeRightPower = Range.clip(-gamepad2.right_trigger, -1.0, 1.0);
            }
            frontMotorLeft.setPower(numFl);
            if (backMotorLeft != null)
                backMotorLeft.setPower(numBl);

            frontMotorRight.setPower(numFr);
            if (backMotorRight != null)
                backMotorRight.setPower(numBr);

            intakeMotorLeft.setPower(intakeLeftPower);
            intakeMotorRight.setPower(intakeRightPower);

            /*Intake Servos
            if (gamepad2.x) {
                servoOffset += SERV_SPEED;
                servoOffset = Range.clip(servoOffset, -0.5, 0.5);
                intakeServoRight.setPosition(MID_SERVO + servoOffset);}
                intakeServoLeft.setPosition(MID_SERVO + servoOffset);
                if (gamepad2.y) {
                    servoOffset -= SERV_SPEED;
                    servoOffset = Range.clip(servoOffset, -0.5, 0.5);
                    intakeServoRight.setPosition(MID_SERVO + servoOffset);
                    intakeServoLeft.setPosition(MID_SERVO + servoOffset);
                }*/

                //Use gamepad2 left & right Bumpers to open and close the claw
                if (gamepad2.right_bumper) {
                    claw.setPosition(-1);
                } else if (gamepad2.left_bumper) {
                    claw.setPosition(1);
                }


                //Rotator.
                if (gamepad2.a) {
                    rotator.setPosition(1);
                }
                if (gamepad2.b) {
                    rotator.setPosition(-1);
                }
                sleep(CYCLE_MS);
                idle();

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                //telemetry.addData("Motors", "left (%.2f), right (%.2f)", intakeLeftPower, intakeRightPower);
                telemetry.update();
            }
        }
    }

