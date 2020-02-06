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

package org.firstinspires.ftc.teamcode.ops.game;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.GameAutoBot;


@Autonomous(name="Blue_Load_Side_Stone", group="game")
//@Disabled
public class Blue_Load_Side_Stone extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private GameAutoBot robot = null;
    private boolean logEnableTrace = true;
    private boolean logToTelemetry = true;


    @Override
    public void runOpMode() {

        robot = new GameAutoBot(this, logEnableTrace, logToTelemetry);
        robot.logger.logInfo("runOpMode", "===== [ Start Initializing ]");

        /* Use either robot.initAll or select only the components that need initializing below */
        robot.initAll();
     /*   robot.driveTrain.init(DriveTrain.InitType.INIT_4WD);
        robot.driveTrainSimple.init();*/


        robot.logger.logInfo("runOpMode", "===== [ Initialization Complete ]");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.logger.logInfo("runOpMode", "===== [ Start Autonomous ]");
        runtime.reset();


        /********** Put Your Code Here **********/

      /*  robot.logger.logInfo("runOpMode", "===== [ Run Forward ]");
        robot.driveTrainSimple.driveByEncoder(0.5, 24);

        robot.logger.logInfo("runOpMode", "===== [ Crab Left ]");
        robot.driveTrainSimple.crabByEncoderLeft(0.5, 24);
        //robot.driveTrainSimple.pause(2);

        robot.logger.logInfo("runOpMode", "===== [ Run Backward ]");
        robot.driveTrainSimple.driveByEncoder(0.5, -24);

        robot.logger.logInfo("runOpMode", "===== [ Crab Right ]");
        robot.driveTrainSimple.crabByEncoderRight(0.5, 24);  */

        robot.driveTrainSimple.crabByEncoderLeft(0.5, 48);
        robot.intake.setIntakePower(0.8);
        robot.driveTrainSimple.driveByEncoder(0.5, 8);
        robot.driveTrainSimple.pause(1);
        robot.intake.setIntakePower(0);
        robot.driveTrainSimple.crabByEncoderRight(0.8, 24);
        robot.driveTrainSimple.driveByEncoder(1, -90);
        robot.driveTrain.gyroRotate(88, .75, true, false);
        robot.driveTrainSimple.driveByEncoder(0.8, -8);
        robot.grapple.grappleMoveDown();
        robot.driveTrain.pause(0.8);
        robot.driveTrainSimple.driveByEncoder(0.8, 24);
        robot.driveTrain.move(1.1, 1, -1);
        robot.grapple.grappleMoveUp();
        robot.driveTrain.moveForward(0.5, .8);
        //slower ramp dumping
        robot.ramp.rampDown(0.8);
        robot.ramp.ramp2Down(0.8);
        sleep(75);
        robot.ramp.rampDown(0.775);
        robot.ramp.ramp2Down(0.775);
        sleep(75);
        robot.ramp.rampDown(0.75);
        robot.ramp.ramp2Down(0.75);
        sleep(75);
        robot.ramp.rampDown(0.725);
        robot.ramp.ramp2Down(0.725);
        sleep(75);
        robot.ramp.rampDown(0.7);
        robot.ramp.ramp2Down(0.7);
        sleep(75);
        robot.ramp.rampDown(0.675);
        robot.ramp.ramp2Down(0.675);
        sleep(75);
        robot.ramp.rampDown(0.65);
        robot.ramp.ramp2Down(0.65);
        sleep(75);
        robot.ramp.rampDown(0.625);
        robot.ramp.ramp2Down(0.625);
        sleep(75);
        robot.ramp.rampDown(0.6);
        robot.ramp.ramp2Down(0.6);
        sleep(75);
        robot.ramp.rampDown(0.575);
        robot.ramp.ramp2Down(0.575);
        sleep(75);
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
        robot.ramp.rampDown(0.35);
        robot.ramp.ramp2Down(0.35);
        sleep(75);
        robot.ramp.rampDown(0.325);
        robot.ramp.ramp2Down(0.325);
        sleep(75);
        robot.ramp.rampDown(0.3);
        robot.ramp.ramp2Down(0.3);
        sleep(200);
        robot.driveTrainSimple.driveByEncoder(0.5, 12);
        robot.ramp.ramp2Up();
        robot.ramp.rampUp();
        robot.driveTrainSimple.driveByEncoder(0.8, 36);
        robot.driveTrainSimple.crabByEncoderLeft(0.5, 12);


        // Show the elapsed game time.
        robot.logger.logInfo("runOpMode", "===== [ Autonomous Complete ] Run Time: %s", runtime.toString());
        telemetry.update();

    }
}
