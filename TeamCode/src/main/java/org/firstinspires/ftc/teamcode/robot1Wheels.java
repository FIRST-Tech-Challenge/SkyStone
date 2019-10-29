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

@TeleOp(name="Mecanumy", group = "RealRobotMecanumWheels")
//@Disabled
public class robot1Wheels extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    @Override
    public void runOpMode() {
        telemetry.addData("status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        //leftFront.setDirection(DcMotor.Direction.FORWARD);
        //leftBack.setDirection(DcMotor.Direction.FORWARD);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        //rightBack.setDirection(DcMotor.Direction.REVERSE);

        double move = gamepad1.left_stick_y;
        double tank = gamepad1.right_stick_x;
        double slide = gamepad1.left_stick_x;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if ( move >= -1 && move < 0 ) {
                leftFront.setDirection(DcMotor.Direction.REVERSE);
                leftBack.setDirection(DcMotor.Direction.REVERSE);
                rightFront.setDirection(DcMotor.Direction.FORWARD);
                rightBack.setDirection(DcMotor.Direction.FORWARD);
                leftFront.setPower(move);
                rightFront.setPower(move);
                leftBack.setPower(move);
                rightBack.setPower(move);
            } else if ( move >0 && move <= 1) {
                leftFront.setDirection(DcMotor.Direction.FORWARD);
                leftBack.setDirection(DcMotor.Direction.FORWARD);
                rightFront.setDirection(DcMotor.Direction.REVERSE);
                rightBack.setDirection(DcMotor.Direction.REVERSE);
                leftFront.setPower(move);
                rightFront.setPower(move);
                leftBack.setPower(move);
                rightBack.setPower(move);
            } else if ( tank <= 1 && tank > 0) {
                leftFront.setDirection(DcMotor.Direction.REVERSE);
                leftBack.setDirection(DcMotor.Direction.REVERSE);
                rightFront.setDirection(DcMotor.Direction.REVERSE);
                rightBack.setDirection(DcMotor.Direction.REVERSE);
                leftFront.setPower(tank);
                rightFront.setPower(tank);
                leftBack.setPower(tank);
                rightBack.setPower(tank);
            } else if ( tank < 0 && tank >= -1 ) {
                leftFront.setDirection(DcMotor.Direction.FORWARD);
                leftBack.setDirection(DcMotor.Direction.FORWARD);
                rightFront.setDirection(DcMotor.Direction.FORWARD);
                rightBack.setDirection(DcMotor.Direction.FORWARD);
                leftFront.setPower(tank);
                rightFront.setPower(tank);
                leftBack.setPower(tank);
                rightBack.setPower(tank);
            } else if ( slide <= 1 && slide > 0) {
                leftBack.setDirection(DcMotor.Direction.REVERSE);
                leftFront.setDirection(DcMotor.Direction.FORWARD);
                rightBack.setDirection(DcMotor.Direction.REVERSE);
                rightFront.setDirection(DcMotor.Direction.FORWARD);
                leftFront.setPower(slide);
                leftBack.setPower(slide);
                rightFront.setPower(slide);
                rightBack.setPower(slide);
            } else if ( slide < 0 && slide > -1) {
                leftBack.setDirection(DcMotor.Direction.FORWARD);
                leftFront.setDirection(DcMotor.Direction.REVERSE);
                rightBack.setDirection(DcMotor.Direction.FORWARD);
                rightFront.setDirection(DcMotor.Direction.REVERSE);
                leftFront.setPower(slide);
                leftBack.setPower(slide);
                rightFront.setPower(slide);
                rightBack.setPower(slide);
            }
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