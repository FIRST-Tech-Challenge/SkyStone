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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//import lines go here. This is just for the program and not for the robot.

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MoveBot_Basic", group="Linear OpMode")
@Disabled
public class MoveBot_Basic extends LinearOpMode {

    //one-time run stage.
    //Declare your unchanging variables here.
    //Put your 'DcMotor leftmotor;' and 'rightmotor' here
    DcMotor rightM;
    DcMotor leftM;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");



        //init stage
        //Declare your changing variables here.
        //put your hardware.map lines here.

        double lPower = 0;
        double rPower = 0;

        double rStick_y = 0;
        double lStick_x = 0;

        double Ky = 0.5;
        double Kx = 0.5;

        leftM = hardwareMap.dcMotor.get("Left_Motor");
        rightM = hardwareMap.dcMotor.get("Right_Motor");

        rightM.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        //init while running
        //Put anything you want to do once before the rest of the program starts looping.
        while(opModeIsActive()) {

            rStick_y = -gamepad1.right_stick_y;
            lStick_x = gamepad1.left_stick_x;

            if(rStick_y != 0) {
                rPower = rStick_y * Ky;
                lPower = rStick_y * Ky;
            } else if(lStick_x < 0) {
                rPower = -lStick_x * Kx;
                lPower =  lStick_x * Kx;

            } else if(lStick_x > 0) {
                rPower = -lStick_x * Kx;
                lPower =  lStick_x * Kx;
            } else {
                rPower = 0;
                lPower = 0;
            }

            leftM.setPower(lPower);
            rightM.setPower(rPower);

            //Code that you want to loop goes here.
        }


    }
}

