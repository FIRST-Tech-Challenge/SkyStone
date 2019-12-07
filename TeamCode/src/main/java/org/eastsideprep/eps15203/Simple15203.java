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

package org.eastsideprep.eps15203;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Fun, Joy, Happiness, and No Toxicity Whatsoever in Team Number 15203 haha that's a good joke lol", group = "15203")

public class Simple15203 extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware15203 robot = new Hardware15203();

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "ready");

        double grabberPos = 0;
        double fingerPos = 0;
        boolean dpad_up_state = false;
        boolean dpad_down_state = false;
        int z_arm_pos_index = 0;
        int[] z_arm_pos = {0, -50, -75, -140};
        double[] z_arm_power = {0.2, 1, 1, 0.4};

        if(z_arm_pos.length != z_arm_power.length) {
            telemetry.addData("ERROR", "z_arm_pos and z_arm_power arrays are not the same length.");
        }

        // Wait for the game to start (driver presses PLAY)
        //robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double left_stick_x = gamepad1.left_stick_x;
            double left_stick_y = gamepad1.left_stick_y;
            double right_stick_x = gamepad1.right_stick_x;
            double right_stick_y = gamepad1.right_stick_y;

            //double orientation = (((double) robot.gyro.getIntegratedZValue()) / 360) * Math.PI * 2;

// wheel commands have two components: drive/strafe and rotation. They have to be weighted.

            double dsAngle = Math.atan2(left_stick_x, left_stick_y);
            double dsWeight = Math.sqrt(left_stick_x * left_stick_x + left_stick_y * left_stick_y);
            double rotPower = right_stick_x;
            double rotWeight = Math.abs(right_stick_x);

            rotPower = rotPower * 0.6;

// correct for the orientation
            //dsAngle -= robot.state.orientation;
            //robot.state.heading = dsAngle;
// rotate more slowly if left trigger pressed
            if (gamepad1.right_bumper) {
                rotPower *= 0.25;
            }
// drive / strafe more slowly if right bumper pressed
            if (gamepad1.left_bumper) {
                dsWeight *= 0.25;
            }

//always drive slower and then go faster when left trigger pressed
            if (gamepad1.left_trigger == 0) {
                dsWeight *= 0.6;
            }


// make sure values are not greater than 1

            if (dsWeight + rotWeight > 1.0) {
                dsWeight /= dsWeight + rotWeight;
                rotPower /= dsWeight + rotWeight;
            }
// finally, do a little math and put them into the motors

            robot.leftFrontMotor.setPower(Math.cos(dsAngle + Math.PI / 4) * dsWeight - rotPower * rotWeight);
            robot.rightBackMotor.setPower(Math.cos(dsAngle + Math.PI / 4) * dsWeight + rotPower * rotWeight);
            robot.rightFrontMotor.setPower(Math.cos(dsAngle - Math.PI / 4) * dsWeight + rotPower * rotWeight);
            robot.leftBackMotor.setPower(Math.cos(dsAngle - Math.PI / 4) * dsWeight - rotPower * rotWeight);

            if(right_stick_x == 0 && right_stick_y == 0 && left_stick_x == 0 && left_stick_y == 0){
                robot.allDrive(0.0, 0);
            }

            if(gamepad1.dpad_up && !dpad_up_state) {
                //if up is pressed, go up a level.
                dpad_up_state = true;
                z_arm_pos_index ++;

                if(z_arm_pos_index > z_arm_pos.length -1 ) {
                    z_arm_pos_index = z_arm_pos.length -1;
                } else {
                    robot.setArmPosition(z_arm_pos[z_arm_pos_index], z_arm_power[z_arm_pos_index]);
                }

            }

            if(gamepad1.dpad_down && !dpad_down_state) {
                //if down is pressed, go down a level.
                dpad_down_state = true;
                z_arm_pos_index --;

                if(z_arm_pos_index < 0) {
                    z_arm_pos_index = 0;
                } else {
                    robot.setArmPosition(z_arm_pos[z_arm_pos_index], z_arm_power[z_arm_pos_index]);
                }
            }

            // idea: use array of positions. use the z_arm_pos array. Use up and down buttons to switch between
            // (keeping count of current index in array with z_arm_pos_index variable.

/* old code
            if(gamepad1.dpad_down && !dpad_down_state) {
                //if down is pressed, go back to resting state.
                dpad_down_state = true;
                robot.setArmPosition(0, 0.5);

                //wait until done moving
                while (robot.zArmMotor.isBusy()) {
                    telemetry.addData("Stuck", "Resetting z-arm position");
                    telemetry.update();
                    //Do nothing
                }
                robot.zArmMotor.setPower(0.0);
                robot.zArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                z_arm_pos_index = 0;

            }
*/
            if(!gamepad1.dpad_up) {
                dpad_up_state = false;
            }

            if(!gamepad1.dpad_down) {
                dpad_down_state = false;
            }

            //grabber controls
            double grabberPosMultiplier = 0.025;

            if(gamepad1.y){
                grabberPos +=  grabberPosMultiplier;
            } else if (gamepad1.a) {
                grabberPos -= grabberPosMultiplier;
            } else {
                grabberPos = robot.grabberServo.getPosition();
            }

            //finger controls
            double fingerPosMultiplier = 0.025;

            if(gamepad1.x){
                fingerPos +=  fingerPosMultiplier;
            } else if (gamepad1.b) {
                fingerPos -= fingerPosMultiplier;
            } else {
                fingerPos = robot.fingerServo.getPosition();
            }

            //Check to make sure the servo isn't hurting itself
            if(grabberPos > 0.4 || robot.grabberServo.getPosition() > 0.4 ) {
                robot.grabberServo.setPosition(0.4);
                grabberPos = 0.4;
            }

            telemetry.addData("Grabber Pos Read", robot.grabberServo.getPosition());
            telemetry.addData("Grabber Pos Var", grabberPos);
            telemetry.addData("Finger Pos Var", fingerPos);
            telemetry.addData("Z arm position index", z_arm_pos_index);
            telemetry.addData("Z arm position array length", z_arm_pos.length);
            telemetry.addData("Z arm position num", robot.zArmMotor.getCurrentPosition());

            telemetry.update();

            //Update the grabber & finger position
            robot.grabberServo.setPosition(grabberPos);
            robot.fingerServo.setPosition(fingerPos);

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);

        }
    }
}
