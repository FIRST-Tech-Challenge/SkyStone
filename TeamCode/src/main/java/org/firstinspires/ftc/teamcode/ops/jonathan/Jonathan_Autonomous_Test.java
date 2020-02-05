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

package org.firstinspires.ftc.teamcode.ops.jonathan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.GameAutoBot;
import org.firstinspires.ftc.teamcode.bots.SimpleBot;
import org.firstinspires.ftc.teamcode.components.DriveTrain;


@Autonomous(name="Jonathan_Autonomous_Test", group="jonathan")
//@Disabled
public class Jonathan_Autonomous_Test extends LinearOpMode {

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

        robot.driveTrainSimple.crabByEncoderRight(0.5, 48);
        robot.intake.setIntakePower(0.8);
        robot.driveTrainSimple.driveByEncoder(0.5, 8);
        robot.driveTrainSimple.pause(1);
        robot.intake.setIntakePower(0);
        robot.driveTrainSimple.driveByEncoder(0.5, -8);
        robot.driveTrainSimple.crabByEncoderLeft(0.5, 24);
        robot.driveTrainSimple.driveByEncoder(0.8, -82);
        robot.driveTrain.gyroRotate(-88, .75, true, false);
        robot.driveTrainSimple.driveByEncoder(0.3, -6);
        robot.grapple.grappleMoveDown();
        robot.driveTrainSimple.pause(1);
        robot.driveTrainSimple.driveByEncoder(0.8, 24);
        robot.driveTrain.move(1.1, -1, 1);
        robot.grapple.grappleMoveUp();
        robot.driveTrainSimple.driveByEncoder(0.8, -48);
        robot.driveTrainSimple.driveByEncoder(0.5, 56);
        robot.driveTrainSimple.crabByEncoderRight(0.5, 12);


        // Show the elapsed game time.
        robot.logger.logInfo("runOpMode", "===== [ Autonomous Complete ] Run Time: %s", runtime.toString());
        telemetry.update();

    }
}
