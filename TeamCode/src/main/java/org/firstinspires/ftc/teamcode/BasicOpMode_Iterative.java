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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.math.*;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic Drive", group="Iterative Opmode")

public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private HardwarePushbot robot = new HardwarePushbot();
    private double speed = 0.5;

    private double leftPos = 0.0;
    private double rightPos = 0.0;
    private double t = 0.0;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        robot.rightArm.setTargetPosition(robot.RIGHT_STOWED);
        robot.leftArm.setTargetPosition(robot.LEFT_STOWED);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightArm.setPower(0.4);
        robot.leftArm.setPower(0.4);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot.rightClaw.setPosition(robot.RIGHT_SERVO_STOWED);
        robot.leftClaw.setPosition(robot.LEFT_SERVO_STOWED);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        robot.mecanumDrive(
                Math.pow(gamepad1.left_stick_y, 3) * speed,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x);

//        if (gamepad1.right_trigger > .1 || gamepad1.left_trigger > .1) {
//            robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            armPower = Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 3);
//        } else {
//            armPower = 0.0;
//        }

        //robot.leftArm.getCurrentPosition();
        //robot.leftArm.setTargetPosition(0);

        if (gamepad1.right_bumper) {
            robot.rightClaw.setPosition(robot.RIGHT_SERVO_CLOSED);
            robot.leftClaw.setPosition(robot.LEFT_SERVO_CLOSED);
        } else if (gamepad1.left_bumper) {
            robot.rightClaw.setPosition(robot.RIGHT_SERVO_OPEN);
            robot.leftClaw.setPosition(robot.LEFT_SERVO_OPEN);
        }

        if (!robot.leftArm.isBusy() && !robot.rightArm.isBusy()) {
            if (gamepad1.a) {
                robot.rightArm.setTargetPosition(robot.RIGHT_GRAB);
                robot.leftArm.setTargetPosition(robot.LEFT_GRAB);
                robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("a", "pressed");
            }

            if (gamepad1.x) {
                robot.rightArm.setTargetPosition(robot.RIGHT_LEV1);
                robot.leftArm.setTargetPosition(robot.LEFT_LEV1);
                robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("x", "pressed");
            }

            if (gamepad1.y) {
                robot.rightArm.setTargetPosition(robot.RIGHT_BRIDGE);
                robot.leftArm.setTargetPosition(robot.LEFT_BRIDGE);
                robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("y", "pressed");
            }

            if (gamepad1.b) {
                robot.rightArm.setTargetPosition(robot.RIGHT_LEV3);
                robot.leftArm.setTargetPosition(robot.LEFT_LEV3);
                robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("b", "pressed");
            }
        }

        robot.rightArm.setPower(1.0);
        robot.leftArm.setPower(1.0);

        telemetry.addData("Left claw pos:", robot.leftClaw.getPosition());
        telemetry.addData("Right claw pos:", robot.rightClaw.getPosition());
        telemetry.addData("Right arm pos:", robot.rightArm.getCurrentPosition());
        telemetry.addData("left arm pos:", robot.leftArm.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
