package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


org.firstinspires.ftc.teamcode ;

        package

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pu.src.main.java.firstinspires.ftc.teamcode.classshbot: Teleop Tank", group="reee")
//@Disabled
public class CodeRed extends OpMode{

    private DcMotor RightWheel = null;
    private DcMotor LeftWheel = null;

            @Override
    public void init() {

       RightWheel = hardwareMap.get(DcMotor.class, "DcMotor_1" );
       LeftWheel = hardwareMap.get(DcMotor.class, "DcMotor_2");

       telemetry.addData("say","Hello driver this is to locally drive the robot with a controller")
            }
            @Override
    public void init_loop() {
            }

            @Override
    public void start() {
            }

            @Override
    public void loop() {

                double left_stick_x;
                double left_trigger;
                double right_trigger;

            left_stick_x = gamepad1.left_stick_x
            left_trigger = gamepad1.left_trigger
            right_trigger = gamepad1. right_trigger


                    RightWheel.setTargetPosition(1);
                    LeftWheel.setTargetPosition(1);

                    if (gamepad1.left_trigger == true ) {
                        telemetry.addLine("left_trigger");
                        RightWheel.RunMode(0.05)
                        LeftWheel.RunMode(-0.05)
                    }
                    else if (gamepad1.right_trigger == true ) {
                        telemetry.addLine("right_trigger");
                        RightWheel.RunMode(-0.05)
                        LeftWheel.RunMode(0.05)

                                else (gamepad1.left_stick_x == true ) {
                            telemetry.addLine("left_stick");
                            {
                                telemetry.addLine("left_stick");
                                RightWheel.RunMode(0.05);
                                LeftWheel.RunMode(0.05);
                            }

                            telemetry.update();


                            @Override
                            public void stop () {
                            }
}