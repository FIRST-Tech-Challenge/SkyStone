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

package org.eastsideprep.murderbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Murderbot: Teleop Simple", group = "Murderbot")

public class MBSimple extends LinearOpMode {

    /* Declare OpMode members. */
    MBHardware robot = new MBHardware();

    @Override
    public void runOpMode() {
        double drive = 0;
        double strafe = 0;
        double rotate = 0;
        double total = 0;




        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Now over wireless");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.state.left_stick_x = gamepad1.left_stick_x;
            robot.state.right_stick_x = gamepad1.right_stick_x;
            robot.state.right_stick_y = gamepad1.right_stick_y;
            robot.state.right_trigger = gamepad1.right_trigger;
            robot.state.orientation = (((double) robot.gyro.getIntegratedZValue()) / 360) * Math.PI * 2;

            // wheel commands have two components: drive/strafe and rotation. They have to be weighted.

            double dsAngle = Math.atan2(robot.state.right_stick_x, robot.state.right_stick_y);
            double dsWeight = Math.sqrt(robot.state.right_stick_x * robot.state.right_stick_x + robot.state.right_stick_y * robot.state.right_stick_y);
            double rotPower = robot.MAX_ROTATION_WEIGHT * robot.state.left_stick_x + robot.state.orientationSweepDelta;
            double rotWeight = robot.MAX_ROTATION_WEIGHT * Math.abs(robot.state.left_stick_x + robot.state.orientationSweepDelta);

            dsAngle -= robot.state.orientation;
            robot.state.heading = dsAngle;

            if (gamepad1.left_bumper) {
                rotPower *= 0.05;
            }

            if (gamepad1.right_bumper) {
                dsWeight *= 0.05;
            }

            if (dsWeight + rotWeight > 1.0) {
                dsWeight /= dsWeight + rotWeight;
                rotPower /= dsWeight + rotWeight;
            }

            robot.leftFrontMotor.setPower(Math.cos(dsAngle + Math.PI / 4) * dsWeight - rotPower * rotWeight);
            robot.rightBackMotor.setPower(Math.cos(dsAngle + Math.PI / 4) * dsWeight + rotPower * rotWeight);
            robot.rightFrontMotor.setPower(Math.cos(dsAngle - Math.PI / 4) * dsWeight + rotPower * rotWeight);
            robot.leftBackMotor.setPower(Math.cos(dsAngle - Math.PI / 4) * dsWeight - rotPower * rotWeight);

            if (gamepad1.right_trigger > 0.1) {
                if (gamepad1.right_trigger > 0.9) {
                    // burst
                    new Thread(new SingleShot(robot, 300)).start();

                } else {
                    // single shot
                    new Thread(new SingleShot(robot, 100)).start();
                }
            }


//            if (robot.avgA0.queryTrigger() == -1) {
//                robot.state.hitCount++;
//            }
            robot.state.hitCount += robot.elf.getHitCount();


            // and now for things requested form other threads that we should only do from this thread

            synchronized (robot.state) {
                if (robot.state.firectrl) {
//                    robot.d0.setState(robot.state.firing);
                    robot.elf.fire(100);
                    robot.state.firectrl = false;
                }
            }


            // reset heading
            if (gamepad1.left_trigger > 0.9) {
                robot.gyro.resetZAxisIntegrator();
            }

            // Send telemetry message to signify robot running;
            telemetry.addLine()
                    .addData("right x", "%.2f", gamepad1.right_stick_x)
                    .addData("right y", "%.2f", gamepad1.right_stick_y)
                    .addData("left x", "%.2f", gamepad1.left_stick_x);
            telemetry.addLine()
                    .addData("dsAngle", "%.2f", dsAngle)
                    .addData("dsWeight", "%.2f", dsWeight)
                    .addData("rotPower", "%.2f", rotPower);
            telemetry.addLine()
                    .addData("orientation", "%.2f", robot.state.orientation)
                    .addData("heading", "%.2f", robot.state.heading);
            telemetry.addLine()
//                    .addData("A0", "%.2f", robot.a0.getVoltage())
//                    .addData("Avg A0", "%.2f", robot.avgA0.getValue())
                    .addData("hitCount", "%d", robot.state.hitCount);

            telemetry.update();

            // Pause for 20 mS each cycle = update 25 times a second.
            sleep(20);
        }
    }
}
