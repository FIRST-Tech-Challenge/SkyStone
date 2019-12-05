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

@TeleOp(name = "Teleop Simple 15203", group = "15203")

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

        int grabberPos;
        int zArmMultiplier;

        // Wait for the game to start (driver presses PLAY)
        //robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double left_stick_x = gamepad1.left_stick_x;
            double left_stick_y = gamepad1.left_stick_y;
            double right_stick_x = gamepad1.right_stick_x;
            double right_stick_y = gamepad1.right_stick_y;

            double right_stick_y_2 = gamepad2.right_stick_y;
            double left_stick_x_2 = gamepad2.left_stick_x;
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
// rotate more slowly if left bumper pressed
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

            if (gamepad1.left_bumper) {
                zArmMultiplier = 3;
            } else {
                zArmMultiplier = 2;
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

            telemetry.addData("Pad 2 Right Stick Y", right_stick_y_2);


            //arm controls
            /*
            if(right_stick_y_2 < 0){
                robot.zArmMotor.setPower(-0.2 * zArmMultiplier);
            } else if (right_stick_y_2 > 0 ) {
                robot.zArmMotor.setPower(0.2 * zArmMultiplier);
            } else {
                robot.zArmMotor.setPower(0.0);
            }
             */

            //grabber controls
            


            telemetry.addData("Grabber Pos", robot.grabberServo.getPosition());
            telemetry.update();
            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);

        }
    }
}
