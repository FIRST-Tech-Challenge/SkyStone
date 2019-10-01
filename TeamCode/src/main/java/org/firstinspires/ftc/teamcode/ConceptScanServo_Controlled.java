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
import com.qualcomm.robotcore.hardware.Servo;
<<<<<<< refs/remotes/origin/Arm
<<<<<<< refs/remotes/origin/Arm
<<<<<<< refs/remotes/origin/Arm
import com.qualcomm.robotcore.hardware.Gamepad;
=======
>>>>>>> Wrote some tests
=======
import com.qualcomm.robotcore.hardware.Gamepad;
>>>>>>> More tests for build team, ready to make a component!
=======
>>>>>>> Revert "More tests for build team, ready to make a component!"

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
<<<<<<< refs/remotes/origin/Arm
<<<<<<< refs/remotes/origin/Arm
<<<<<<< refs/remotes/origin/Arm
@TeleOp(name = "Concept: controlled arm", group = "Concept")
public class ConceptScanServo_Controlled extends LinearOpMode {
=======
@TeleOp(name = "Concept: Get servo position", group = "Concept")
public class ConceptScanServo extends LinearOpMode {
>>>>>>> Wrote some tests
=======
@TeleOp(name = "Concept: controlled arm", group = "Concept")
public class ConceptScanServo_Controlled extends LinearOpMode {
>>>>>>> More tests for build team, ready to make a component!
=======
@TeleOp(name = "Concept: Get servo position", group = "Concept")
public class ConceptScanServo extends LinearOpMode {
>>>>>>> Revert "More tests for build team, ready to make a component!"

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position, "%5.2f", servo.getDirection()
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
<<<<<<< refs/remotes/origin/Arm
<<<<<<< refs/remotes/origin/Arm
<<<<<<< refs/remotes/origin/Arm
    Servo gripper;
    Servo wrist;
    Servo elbow;
=======
    Servo   servo;
>>>>>>> Wrote some tests
=======
    Servo gripper;
    Servo wrist;
    Servo elbow;
>>>>>>> More tests for build team, ready to make a component!
=======
    Servo   servo;
>>>>>>> Revert "More tests for build team, ready to make a component!"
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;


    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
<<<<<<< refs/remotes/origin/Arm
<<<<<<< refs/remotes/origin/Arm
<<<<<<< refs/remotes/origin/Arm
        gripper = hardwareMap.get(Servo.class, "gripper");
        wrist = hardwareMap.get(Servo.class, "wrist");
=======
        servo = hardwareMap.get(Servo.class, "left_hand");

>>>>>>> Wrote some tests
=======
        gripper = hardwareMap.get(Servo.class, "gripper");
        wrist = hardwareMap.get(Servo.class, "wrist");
>>>>>>> More tests for build team, ready to make a component!
=======
        servo = hardwareMap.get(Servo.class, "left_hand");

>>>>>>> Revert "More tests for build team, ready to make a component!"
        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
<<<<<<< refs/remotes/origin/Arm
<<<<<<< refs/remotes/origin/Arm
<<<<<<< refs/remotes/origin/Arm
        float gripperPos = 0;
        while(opModeIsActive()){
            // Display the current value
            try {
                if (gamepad1.a && gripperPos <= 1) {
                    gripperPos += 0.0001;
                } else if (gamepad1.b && gripperPos >= 0){
                    gripperPos -= 0.0001;
                }
                gripper.setPosition(gripperPos);
                telemetry.addData("Gripper Position: " + gripper.getPosition(), "");
                telemetry.addData("Wrist position: " + wrist.getPosition(), "");
=======
        while(opModeIsActive()){
            // Display the current value
            try {
                telemetry.addData("Servo Position: " + servo.getPosition() + Double.toString(Math.random()), "");
>>>>>>> Wrote some tests
=======
        float gripperPos = 0;
        while(opModeIsActive()){
            // Display the current value
            try {
                if (gamepad1.a && gripperPos <= 1) {
                    gripperPos += 0.0001;
                } else if (gamepad1.b && gripperPos >= 0){
                    gripperPos -= 0.0001;
                }
                gripper.setPosition(gripperPos);
                telemetry.addData("Gripper Position: " + gripper.getPosition(), "");
                telemetry.addData("Wrist position: " + wrist.getPosition(), "");
>>>>>>> More tests for build team, ready to make a component!
=======
        while(opModeIsActive()){
            // Display the current value
            try {
                telemetry.addData("Servo Position: " + servo.getPosition() + Double.toString(Math.random()), "");
>>>>>>> Revert "More tests for build team, ready to make a component!"
                telemetry.addData(">", "Press Stop to end test." );
                telemetry.update();
            } catch (Exception e) {
                telemetry.addData("oopsie whoopsie", "");
                telemetry.update();
            }

        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
