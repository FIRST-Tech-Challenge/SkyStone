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

package org.firstinspires.ftc.teamcode.ops.Auto_Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.TestBot;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.ConceptTensorFlowObjectDetectionWebcam;


@Autonomous(name="WebcamTest", group="Auto_Tests")
//@Disabled
public class Webcam_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TestBot robot = null;
    private boolean logEnableTrace = true;
    private boolean logToTelemetry = true;


    @Override
    public void runOpMode() {

        robot = new TestBot(this, logEnableTrace, logToTelemetry);
        robot.logger.logInfo("runOpMode", "===== [ Start Initializing ]");

        /* Use either robot.initAll or select only the components that need initializing below */
        //robot.initAll();
        robot.driveTrain.init(DriveTrain.InitType.INIT_4WD);
        robot.gyroNavigator.init();
        //robot.colorDetection.initColor();
        //robot.gyroNavigator2.init();

        robot.logger.logInfo("runOpMode", "===== [ Initialization Complete ]");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.logger.logInfo("runOpMode", "===== [ Start Autonomous ]");
        runtime.reset();

            double angle1 = robot.gyroNavigator.getAngle();
         //   double angle2 = robot.gyroNavigator2.getAngle();

         /*  robot.driveTrain.encoderDrive(1, -0.5);
           robot.driveTrain.gyroRotate(90, 0.5, true, false);
           robot.driveTrain.encoderDrive(1, -40);
           robot.driveTrain.gyroRotate(-90, 0.5, true, false);
           robot.driveTrain.encoderDrive(1, -10);
           robot.driveTrain.encoderDrive(1, 10); */
          while(robot.skystoneFinder.canSeeSkystone() == true){
              robot.driveTrain.moveForward(1,1);
          }



            //robot.logger.logInfo("runOpMode", "Angles: 1:%f", angle1);

      //  }

        // Show the elapsed game time.
        robot.logger.logInfo("runOpMode", "===== [ Autonomous Complete ] Run Time: %s", runtime.toString());
        telemetry.update();

    }
}
