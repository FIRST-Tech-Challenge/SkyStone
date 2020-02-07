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

package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.*;

public class ColorBot extends Bot {

    /* BotComponents */

    public Logger logger = null;
    public DriveTrain driveTrain = null;
    public DriveTrainSimple driveTrainSimple = null;
    public ColorFinder colorFinder = null;
    public Intake intake = null;


    /* Constructor */
    public ColorBot() {

    }

    public ColorBot(OpMode aOpMode) {
        this(aOpMode, false, false);
    }

    public ColorBot(OpMode aOpMode, boolean enableTrace, boolean enableTelemetry) {

        logger = new Logger("SimpleBot", aOpMode, enableTrace, enableTelemetry);
        driveTrain = new DriveTrain(logger, aOpMode, "frontLeftMotor", "frontRightMotor",
                "backLeftMotor", "backRightMotor");

        driveTrainSimple = new DriveTrainSimple(logger, aOpMode, "frontLeftMotor", "frontRightMotor",
                "backLeftMotor", "backRightMotor");

        colorFinder = new ColorFinder(logger, aOpMode, "leftColor", "rightColor");

        intake = new Intake(logger, aOpMode, "Right_Intake", "Left_Intake");


    }

    public void initAll() {
        driveTrain.init(DriveTrain.InitType.INIT_4WD);
        driveTrainSimple.init();
        colorFinder.init();

    }

}

