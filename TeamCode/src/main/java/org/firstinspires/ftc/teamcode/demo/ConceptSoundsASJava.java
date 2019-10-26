/* Copyright (c) 2018 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.demo;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

/**
 * This file demonstrates how to play simple sounds on both the RC and DS phones.
 * It illustrates how to build sounds into your application as a resource.
 * This technique is best suited for use with Android Studio since it assumes you will be creating a new application
 *
 * If you are using OnBotJava, please see the ConceptSoundsOnBotJava sample
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * Operation:
 *
 * Gamepad X & B buttons are used to trigger sounds in this example, but any event can be used.
 * Note: Time should be allowed for sounds to complete before playing other sounds.
 *
 * For sound files to be used as a compiled-in resource, they need to be located in a folder called "raw" under your "res" (resources) folder.
 * You can create your own "raw" folder from scratch, or you can copy the one from the FtcRobotController module.
 *
m *       <project root>/TeamCode/src/main/res/raw
 *
 *     Copy any .wav files you want to play into this folder.
 *     Make sure that your files ONLY use lower-case characters, and have no spaces or special characters other than underscore.
 *
 *     The name you give your .wav files will become the resource ID for these sounds.
 *     eg:  gold.wav becomes R.raw.gold
 *
 *     If you wish to use the sounds provided for this sample, they are located in:
 *     <project root>/FtcRobotController/src/main/res/raw
 *     You can copy and paste the entire 'raw' folder using Android Studio.
 *
 */

@TeleOp(name="don't ask who joe is", group="Concept")
public class ConceptSoundsASJava extends LinearOpMode {

    // Declare OpMode members.
    private boolean bruh;      // Sound file present flags

    private boolean isX = false;    // Gamepad button state variables

    private boolean wasX = false;   // Gamepad button history variables

    @Override
    public void runOpMode() {

        // Determine Resource IDs for sounds built into the RC application.
        int bruhSoundID = hardwareMap.appContext.getResources().getIdentifier("bruh", "raw", hardwareMap.appContext.getPackageName());

        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.

        if (bruhSoundID != 0)
            bruh = SoundPlayer.getInstance().preload(hardwareMap.appContext, bruhSoundID);

        // Display sound status
        telemetry.addData("SafetyPatrol", bruh ? "Found" : "Not found\n Add bruh.wav to /src/main/res/raw" );

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Start to glimpse nirvana");
        telemetry.update();
        waitForStart();

        telemetry.addData(">", "Press X to be reborn");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // say Silver each time gamepad X is pressed (This sound is a resource)
            if (bruh && (isX = gamepad1.x) && !wasX) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, bruhSoundID);
                telemetry.addData("Having", "a Bruh Moment");
                telemetry.update();
            }


            // Save last button states
            wasX = isX;
        }
    }
}
