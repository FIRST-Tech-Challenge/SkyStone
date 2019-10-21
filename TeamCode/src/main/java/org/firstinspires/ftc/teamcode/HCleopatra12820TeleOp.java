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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode uses the common Cleopatra hardware class to define the devices on the robot.
 * All device access is managed through the CleopatraHardware class.
 * The code is structured as a LinearOpMode
 *

 */

@TeleOp(name="Hardware Cleopatra Tele_Op", group="Tele_Op")
//Disabled
public class HCleopatra12820TeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    CleopatraHardware robot           = new CleopatraHardware();   // It uses Cleopatra's hardware
    double          clawOffset;                       // Servo mid position
    final double    CLAW_SPEED; // sets rate to move claw servo
    final double INCREMENT;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS;     // period of each cycle
    //static final double MAX_POS;     // Maximum rotational position
    //static final double MIN_POS; // Minimum rotational position

    static {
        CYCLE_MS = 50;
        //MAX_POS = 1.0;
        //MIN_POS = 0.0;
    }

    public HCleopatra12820TeleOp() {
        INCREMENT = 0.01;
        CLAW_SPEED = 0.02;
        clawOffset = 0;
    }
    // Define class members
    //Servo servo;
    double  position = robot.MID_SERVO;// Start at halfway position
    //boolean rampUp = true;

    @Override
    public void runOpMode() {



         /* The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double intakeLeftPower;
            double intakeRightPower;
            double Speed = -gamepad1.left_stick_y;
            double Turn = gamepad1.left_stick_x;
            double Strafe = gamepad1.right_stick_x;
            double MAX_SPEED = 1.0;

            /*
      Front Left= +Speed + Turn - Strafe      Front Right= +Speed - Turn + Strafe
      Back Left  = +Speed + Turn + Strafe      Back Right = +Speed - Turn - Strafe
*/
            double numFl = Range.clip((+Speed + Turn - Strafe), -1, +1);
            double numBl = Range.clip((+Speed + Turn + Strafe), -1, 1);
            double numFr = Range.clip((+Speed - Turn + Strafe), -1, +1);
            double numBr = Range.clip((+Speed - Turn - Strafe), -1, 1);
            //double armElbowPower = Range.clip(0.5*gamepad2.left_stick_y, -1, 1);

            robot.frontMotorLeft.setPower(numFl -MAX_SPEED +MAX_SPEED);
            if (robot.backMotorLeft!= null) {
                robot.backMotorLeft.setPower(numBl -MAX_SPEED +MAX_SPEED);
            }
            robot.frontMotorRight.setPower(numFr -MAX_SPEED +MAX_SPEED);
            if (robot.backMotorRight != null) {
                robot.backMotorRight.setPower(numBr -MAX_SPEED +MAX_SPEED);
            }
            //Rotator.
            if(gamepad1.a)
                position += INCREMENT ;
            if(gamepad1.b)
                position-=INCREMENT;
            robot.rotator.setPosition(position);
            sleep(CYCLE_MS);
            idle();

            //Arm wrist
            if(gamepad2.a)
            robot.armWrist.setPower(robot.ARM_UP_POWER);

            else if (gamepad2.b)
                robot.armWrist.setPower(robot.ARM_DOWN_POWER);

            else
                robot.armWrist.setPower(0.0);

            //Arm Elbow
            if(gamepad2.dpad_up)
                robot.armElbow.setPower(robot.ELBOW_UP_POWER);

            else if (gamepad2.dpad_down)
                robot.armElbow.setPower(robot.ELBOW_DOWN_POWER);
            else
                robot.armElbow.setPower(0.0);

            //Intake
            if(gamepad2.left_trigger>0 ){
                intakeLeftPower    = Range.clip(gamepad2.left_trigger, -1.0, 1.0) ;
                intakeRightPower   = Range.clip(gamepad2.left_trigger, -1.0, 1.0) ;}
            else{
                intakeLeftPower    = Range.clip(-gamepad2.right_trigger, -1.0, 1.0) ;
                intakeRightPower   = Range.clip(-gamepad2.right_trigger, -1.0, 1.0) ;}
            robot.intakeMotorLeft.setPower(intakeLeftPower);
            robot.intakeMotorRight.setPower(intakeRightPower);

            //Intake Servos
            if (gamepad2.x){
                //intakeServoRight.setPosition(srvPower);
                robot.intakeServoRight.setPosition(1);
                robot.intakeServoLeft.setPosition(1);
            }
            if(gamepad2.y){
                robot.intakeServoRight.setPosition(0);
                robot.intakeServoLeft.setPosition(0);

            }
            //Use gamepad2 left & right Bumpers to open and close the claw
            if(gamepad2.right_bumper)
                clawOffset += CLAW_SPEED;

            else if(gamepad2.left_bumper)
                robot.claw.setPosition(0);
                clawOffset -= CLAW_SPEED;

            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            robot.claw.setPosition(robot.MID_SERVO + clawOffset);

            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", robot.intakeMotorLeft);
            telemetry.addData("right", "%.2f", robot.intakeMotorRight);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
