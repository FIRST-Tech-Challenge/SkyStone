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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bots.ColorBot;

import java.util.Locale;


@Autonomous(name="Jonathan_Auto_Color_Test", group="jonathan")
//@Disabled
public class Jonathan_Auto_Color_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ColorBot robot = null;
    private boolean logEnableTrace = true;
    private boolean logToTelemetry = true;


    @Override
    public void runOpMode() {

        robot = new ColorBot(this, logEnableTrace, logToTelemetry);
        robot.logger.logInfo("runOpMode", "===== [ Start Initializing ]");

        /* Use either robot.initAll or select only the components that need initializing below */
        robot.initAll();

        robot.logger.logInfo("runOpMode", "===== [ Initialization Complete ]");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.logger.logInfo("runOpMode", "===== [ Start Autonomous ]");
        runtime.reset();

        robot.driveTrainSimple.resetEncoders();

        double speedFactor = 0.5;
        double increment = 0.1;


        /********** Put Your Code Here **********/
        if (robot.colorFinder.isAvailable) {

            double distance = robot.colorFinder.rightDistance.getDistance(DistanceUnit.CM);

            while (opModeIsActive() && distance > (double)4.0) {

                if (speedFactor < 1) {
                    speedFactor += increment;
                    robot.driveTrainSimple.setPower(speedFactor, -0.25, 0.30, 0.20, -.25);
                }

                distance = robot.colorFinder.rightDistance.getDistance(DistanceUnit.CM);


                robot.logger.logInfo("runOpMode", "distance: %f", distance);


                // send the info back to driver station using telemetry function.
                telemetry.addData("Distance (cm)",
                        String.format(Locale.US, "%.02f", distance));

                telemetry.addData("Alpha", robot.colorFinder.rightColor.alpha());
                telemetry.addData("Red  ", robot.colorFinder.rightColor.red());
                telemetry.addData("Green", robot.colorFinder.rightColor.green());
                telemetry.addData("Blue ", robot.colorFinder.rightColor.blue());

                telemetry.update();


            }

            robot.driveTrainSimple.stop();

        }


        // Show the elapsed game time.
        robot.logger.logInfo("runOpMode", "===== [ Autonomous Complete ] Run Time: %s", runtime.toString());
        telemetry.update();

    }

    private void waitForButton() {
        while (opModeIsActive() && gamepad1.y == false) {
            robot.driveTrain.stop();
        }
    }
}
