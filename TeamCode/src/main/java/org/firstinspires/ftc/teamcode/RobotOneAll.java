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
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "ROneAll", group = "R1")
//@Disabled
public class RobotOneAll extends LinearOpMode {
    RobotOneHardware robotOne           = new RobotOneHardware();   // Use a Pushbot's hardware


    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        boolean grab;

        robotOne.init(hardwareMap);

        telemetry.addData("Say", "Goonies Never Say Die");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = gamepad1.left_stick_y;
            turn  = -gamepad1.left_stick_x;
            //grab = gamepad1.left_bumper;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            robotOne.leftDrive.setPower(left);
            robotOne.rightDrive.setPower(right);
//            if (grab == 1) {
//                robotOne.leftServo.setPosition(0.5);
//                robotOne.rightServo.setPosition(0.5);
//            }
            // Use gamepad left & right Bumpers to open and close the claw
            /*if (gamepad1.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad1.left_bumper)
                clawOffset -= CLAW_SPEED;
*/
            // Move both servos to new position.  Assume servos are mirror image of each other.
            //clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            //robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
            //robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad1.a) {
                robotOne.InAndOut.setPower(.1);
                sleep(1000);
                robotOne.InAndOut.setPower(.1);
            }
            else if (gamepad1.y) {
                robotOne.InAndOut.setPower(-.05);
                sleep(100);
                //robotOne.InAndOut.setPower(-.2);
                sleep(50);
                robotOne.InAndOut.setPower(-.1);
                sleep(1000);
                robotOne.InAndOut.setPower(1);
                sleep(200);
                robotOne.InAndOut.setPower(0);
                /*sleep(100);
                robotOne.InAndOut.setPower(.7);
                sleep(500);*/
                /*robotOne.InAndOut.setPower(.5);
                sleep(1000);
                robotOne.InAndOut.setPower(.4);
                sleep(2000);*/
            }
            else
                robotOne.InAndOut.setPower(0.0);

            // Send telemetry message to signify robot running;
            //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();



            if (gamepad1.left_bumper) {
                robotOne.leftServo.setPosition(0.3);
                robotOne.rightServo.setPosition(1.05);
            }

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }



        }
    }
