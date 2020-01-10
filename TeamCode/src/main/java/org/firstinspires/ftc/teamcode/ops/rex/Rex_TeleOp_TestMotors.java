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

package org.firstinspires.ftc.teamcode.ops.rex;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.*;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.WebCamera;

@TeleOp(name="Rex_TeleOp_TestMotors", group="rex")
//@Disabled
public class Rex_TeleOp_TestMotors extends LinearOpMode
{
    // Declare OpMode members.
    private SimpleBot robot       = null;
    private ElapsedTime runtime = new ElapsedTime();
    private Boolean activateFrontMotors = false;
    private Boolean activateBackMotors = false;
    private boolean logEnableTrace = true;
    private boolean logToTelemetry = true;


    @Override
    public void runOpMode() {

        robot = new SimpleBot (this, logEnableTrace, logToTelemetry);
        robot.logger.logInfo("runOpMode", "===== [ Start Initializing ]");

        robot.driveTrain.init(DriveTrain.InitType.INIT_4WD);

        robot.logger.logInfo("runOpMode", "===== [ Initialization Complete ]");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.logger.logInfo("runOpMode", "===== [ Start TeleOp ]");
        runtime.reset();

        while (opModeIsActive()) {


            if (robot.driveTrain.isAvailable) {
                if (gamepad1.dpad_down) {
                    robot.logger.logInfo("runOpMode", "===== [ Encoder Drive Forward ]");
                    robot.driveTrain.encoderDrive(.5, -1);
                } else if (gamepad1.dpad_up) {
                    robot.logger.logInfo("runOpMode", "===== [ Encoder Drive Backward ]");
                    robot.driveTrain.encoderDrive(.5, 1);
                } else {
                    robot.driveTrain.stop();
                }
            }

            if (gamepad1.y) {
                activateFrontMotors = true;
                robot.logger.logInfo("runOpMode", "===== [ Front Motors : Activate ]");
            }

            if (gamepad1.b) {
                activateBackMotors = true;
                robot.logger.logInfo("runOpMode", "===== [ Back Motors : Activate ]");
            }

            if (gamepad1.x) {
                activateFrontMotors = false;
                robot.logger.logInfo("runOpMode", "===== [ Front Motors : OFF ]");
            }

            if (gamepad1.a) {
                activateBackMotors = false;
                robot.logger.logInfo("runOpMode", "===== [ Back Motors : OFF ]");
            }


            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;
            double rightY = gamepad1.right_stick_y;

            if (robot.driveTrain.isAvailable) {
                if (activateFrontMotors) {
                    robot.driveTrain.frontLeftMotor.setPower(leftY);
                    robot.driveTrain.frontRightMotor.setPower(rightY);
                } else {
                    robot.driveTrain.frontLeftMotor.setPower(0);
                    robot.driveTrain.frontRightMotor.setPower(0);
                }

                if (activateBackMotors) {
                    robot.driveTrain.backLeftMotor.setPower(leftY);
                    robot.driveTrain.backRightMotor.setPower(rightY);
                } else {
                    robot.driveTrain.backLeftMotor.setPower(0);
                    robot.driveTrain.backRightMotor.setPower(0);
                }
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left", "X (%.2f), Y (%.2f)", leftX, leftY);
            telemetry.addData("Right", "X (%.2f), Y (%.2f)", rightX, rightY);


            telemetry.addData("Front Motors", "%b", activateFrontMotors);

            if (robot.driveTrain.isAvailable) {
                double frontLPower = robot.driveTrain.frontLeftMotor.getPower();
                double frontRPower = robot.driveTrain.frontRightMotor.getPower();
                telemetry.addData("Front", "L (%.2f), R (%.2f)", frontLPower, frontRPower);
            }

            telemetry.addData("Back Motors", "%b", activateBackMotors);

            if (robot.driveTrain.isAvailable) {
                double backLPower = robot.driveTrain.backLeftMotor.getPower();
                double backRPower = robot.driveTrain.backRightMotor.getPower();
                telemetry.addData("Back", "L (%.2f), R (%.2f)", backLPower, backRPower);
            }

            telemetry.update();
        }

    }

}
