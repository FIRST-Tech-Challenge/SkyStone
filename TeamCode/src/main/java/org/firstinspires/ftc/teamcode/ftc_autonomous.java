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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name="ftc_autonomous", group="Linear Opmode")
//Disabled
public class ftc_autonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    private DcMotor leftback = null;
    //private DcMotor Arm = null;
    private DcMotor rightback = null;

    private double drivepower = 0.0;
    private double right = 0.0;
    private double left = 0.0;


    private void lateralmovement() {
        leftfront.setPower(drivepower);
        rightfront.setPower(drivepower);
        rightback.setPower(drivepower);
        leftback.setPower(drivepower);
    }

    private void lateralright() {
        leftfront.setPower(right);
        rightfront.setPower(-right);
        rightback.setPower(right);
        leftback.setPower(-right);
    }

    private void lateralleft() {
        leftfront.setPower(-left);
        rightfront.setPower(left);
        rightback.setPower(-left);
        leftback.setPower(left);
    }


    //Define class members
    Servo   servo;
    double  servoPosition = 0.4; // Start at halfway position
    boolean rampUp = true;



        public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        servo = hardwareMap.servo.get("servo");
        servo.setPosition(servoPosition);

        // Initialize the hardware variables. Note that the strings used here as parameters
        leftfront  = hardwareMap.get(DcMotor.class, "leftfront");
        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        rightback = hardwareMap.get(DcMotor.class, "rightback");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        // Arm  = hardwareMap.get(DcMotor.class, "Arm");
        // boxmotor = hardwareMap.get(Servo.class, "boxmotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        //Arm.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        // Arm.setPower(1);
        //sleep(2500);
        //Arm.setPower(0);


        //drive forward for 2 seconds
        drivepower = -0.5;
        lateralmovement();
        telemetry.addData("Status", "Moving Backword");
        telemetry.update();
        sleep(1760);

        //stop motors
        drivepower = 0.0;
        lateralmovement();
        telemetry.addData("Status", "Stopping");
        telemetry.update();

        // servomotors
        servoPosition = 0.95;
        servo.setPosition(servoPosition);
        sleep(900);

        //drive backwards for 2 seconds
        drivepower = 0.5;
        lateralmovement();
        telemetry.addData("Status", "Moving Backwards");
        telemetry.update();
        sleep(2300);

        drivepower = 0.0;
        lateralmovement();
        telemetry.addData("Status", "Stop Program");
        telemetry.update();

        servoPosition = 0.4;
        servo.setPosition(servoPosition);
        sleep(500);
        drivepower = 0.0;



        lateralmovement();
        telemetry.addData("Status", "Stop Program");
        telemetry.update();
    }
}



