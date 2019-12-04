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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */
@Disabled
public abstract class BaseAutoOpMode extends BaseOpMode {

    public void UnfoldRobot() {

        front_left.setPower(.8);
        front_right.setPower(.8);
        rear_left.setPower(.8);
        rear_right.setPower(.8);

        sleep(200);

        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);
        telemetry.addData("Encoder test", front_left.getCurrentPosition());
        telemetry.update();

        sleep(500);

        /*lift_left.setPower(1);
        lift_right.setPower(1);
        sleep(200)

        lift_left.setPower(0);
        lift_right.setPower(0);
        sleep(200);


        Clamp_Left.setPosition(0.73f);
        Clamp_Right.setPosition(0.27f);
        sleep(250);

         */

        lift_left.setPower(-1);
        lift_right.setPower(-1);
        sleep(500);

        lift_left.setPower(0);
        lift_right.setPower(0);
        sleep(200);

        //Release_Servo.setPosition(1);
        //sleep(200);
        //Release_Servo.setPosition(0);
        //sleep(100);

        lift_left.setPower(-1);
        lift_right.setPower(-1);
        sleep(300);

        lift_left.setPower(0);
        lift_right.setPower(0);
        sleep(200);

        front_left.setPower(.8);
        front_right.setPower(.8);
        rear_left.setPower(.8);
        rear_right.setPower(.8);
        sleep(200);

        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);
        telemetry.addData("Encoder test", front_left.getCurrentPosition());
        telemetry.update();
    }


    public void CutMotors() {
        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);
    }
    public void RunAllMotors() {
        front_left.setPower(1);
        front_right.setPower(1);
        rear_left.setPower(1);
        rear_right.setPower(1);
    }
    public void RunAllMotorsBackward() {
        front_left.setPower(-1);
        front_right.setPower(-1);
        rear_left.setPower(-1);
        rear_right.setPower(-1);
    }
}





