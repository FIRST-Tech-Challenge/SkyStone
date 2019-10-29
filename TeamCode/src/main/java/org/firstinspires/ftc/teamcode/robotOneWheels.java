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

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ROneDrive", group = "robotOneWheels")
//@Disabled
public class robotOneWheels extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() {
        telemetry.addData("status", "Initialized");
        telemetry.update();

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        //leftFront.setDirection(DcMotor.Direction.FORWARD);
        //leftBack.setDirection(DcMotor.Direction.FORWARD);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        //rightBack.setDirection(DcMotor.Direction.REVERSE);

        double move = gamepad1.right_stick_y;
        double turn = gamepad1.right_stick_x;


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if ( move >= -1 && move < 0 ) {
                leftDrive.setDirection(DcMotor.Direction.REVERSE);
                rightDrive.setDirection(DcMotor.Direction.FORWARD);
                leftDrive.setPower(move);
                rightDrive.setPower(move);

            } else if ( move >0 && move <= 1) {
                leftDrive.setDirection(DcMotor.Direction.FORWARD);
                rightDrive.setDirection(DcMotor.Direction.REVERSE);
                leftDrive.setPower(move);
                rightDrive.setPower(move);
            } else if ( turn <= 1 && turn > 0) {
                leftDrive.setDirection(DcMotor.Direction.REVERSE);
                rightDrive.setDirection(DcMotor.Direction.REVERSE);
                leftDrive.setPower(turn);
                rightDrive.setPower(turn);
            } else if ( turn < 0 && turn >= -1 ) {
                leftDrive.setDirection(DcMotor.Direction.FORWARD);
                rightDrive.setDirection(DcMotor.Direction.FORWARD);
                leftDrive.setPower(turn);
                rightDrive.setPower(turn);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", InAndOut);
            telemetry.update();

//            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)
//            double
//            double rightx
//            double leftx
//            double lefty
//            final double v1 =
//            final double v2 =
//            final double v3 =
//            final double v4 =
        }
    }
}