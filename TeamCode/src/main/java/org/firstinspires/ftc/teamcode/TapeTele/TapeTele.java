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

package org.firstinspires.ftc.teamcode.TapeTele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.TestBot;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.TapeGun;


@TeleOp(name="tapeGunMan", group="game")
public class TapeTele extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TestBot robot = null;
    private boolean logEnableTrace = false;
    private boolean logToTelemetry = true;


    @Override
    public void runOpMode() {

        robot = new TestBot(this, logEnableTrace, logToTelemetry);
        robot.logger.logInfo("runOpMode", "===== [ Start Initializing ]");

        /* Use either robot.initAll or select only the components that need initializing below */
        //robot.initAll();
        robot.driveTrain.init(DriveTrain.InitType.INIT_4WD);

        robot.logger.logInfo("runOpMode", "===== [ Initialization Complete ]");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.grapple.init();
        robot.ramp.init();
        robot.intake.init();
        robot.tapeGun.init();

        robot.logger.logInfo("runOpMode", "===== [ Start TeleOp ]");
        runtime.reset();

        while (opModeIsActive()) {


            /********** Put Your Code Here **********/
            if (robot.driveTrain.isAvailable) {
                double leftX = gamepad1.left_stick_x;
                double leftY = gamepad1.left_stick_y;
                double rightX = gamepad1.right_stick_x;
                double rightY = gamepad1.right_stick_y;

                robot.driveTrain.updateMotorsMechanumDrive(leftX, leftY, rightX, rightY);

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Left", "X (%.2f), Y (%.2f)", leftX, leftY);
                telemetry.addData("Right", "X (%.2f), Y (%.2f)", rightX, rightY);

            }

            if (robot.intake.isAvailable) {
                if (gamepad1.left_trigger > 0) {
                    robot.intake.setIntakePower(-1);
                }
                if(gamepad1.left_trigger <= 0)
                {
                    robot.intake.setIntakePower(0);
                }
            }


            if (robot.intake.isAvailable) {
                if (gamepad1.right_trigger > 0) {
                    robot.intake.setIntakePower(0.9);
                }
                if(gamepad1.right_trigger <= 0)
                {
                    robot.intake.setIntakePower(0);
                }
            }

            if(gamepad1.left_bumper){
                robot.ramp.rampUp();
                robot.ramp.ramp2Up();
            }
            if(gamepad1.right_bumper){
                robot.ramp.rampDown(0.8);
                robot.ramp.ramp2Down(0.8);
                sleep(50);
                robot.ramp.rampDown(0.775);
                robot.ramp.ramp2Down(0.775);
                sleep(50);
                robot.ramp.rampDown(0.75);
                robot.ramp.ramp2Down(0.75);
                sleep(50);
                robot.ramp.rampDown(0.725);
                robot.ramp.ramp2Down(0.725);
                sleep(50);
                robot.ramp.rampDown(0.7);
                robot.ramp.ramp2Down(0.7);
                sleep(50);
                robot.ramp.rampDown(0.675);
                robot.ramp.ramp2Down(0.675);
                sleep(50);
                robot.ramp.rampDown(0.65);
                robot.ramp.ramp2Down(0.65);
                sleep(50);
                robot.ramp.rampDown(0.625);
                robot.ramp.ramp2Down(0.625);
                sleep(50);
                robot.ramp.rampDown(0.6);
                robot.ramp.ramp2Down(0.6);
                sleep(50);
                robot.ramp.rampDown(0.575);
                robot.ramp.ramp2Down(0.575);
                sleep(50);
                robot.ramp.rampDown(0.55);
                robot.ramp.ramp2Down(0.55);
                sleep(75);
                robot.ramp.rampDown(0.525);
                robot.ramp.ramp2Down(0.525);
                sleep(75);
                robot.ramp.rampDown(0.5);
                robot.ramp.ramp2Down(0.5);
                sleep(75);
                robot.ramp.rampDown(0.475);
                robot.ramp.ramp2Down(0.475);
                sleep(75);
                robot.ramp.rampDown(0.45);
                robot.ramp.ramp2Down(0.45);
                sleep(75);
                robot.ramp.rampDown(0.425);
                robot.ramp.ramp2Down(0.425);
                sleep(75);
                robot.ramp.rampDown(0.4);
                robot.ramp.ramp2Down(0.4);
                sleep(75);
                robot.ramp.rampDown(0.375);
                robot.ramp.ramp2Down(0.375);
                sleep(75);
                robot.ramp.rampDown(0.36);
                robot.ramp.ramp2Down(0.36);
                sleep(75);

            }

            if(gamepad1.x){
                robot.grapple.grappleMoveDown();
            }
            if (gamepad1.y){



                robot.grapple.grappleMoveUp();
            }

            if(gamepad1.a){
                robot.tapeGun.extendTape();
            }
            if(gamepad1.b){
                robot.tapeGun.suckTape();
            }
            else{
                robot.tapeGun.stopTape();
            }

            // Show the elapsed game time.
            robot.logger.logInfo("runOpMode", "===== [ TeleOp Complete ] Run Time: %s", runtime.toString());
            telemetry.update();

        }
    }
}