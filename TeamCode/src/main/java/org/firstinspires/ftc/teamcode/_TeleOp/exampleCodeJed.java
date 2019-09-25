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

package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collections;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@TeleOp(name="Jed Mode", group="Iterative Opmode")
public class exampleCodeJed extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;
    // Declare connection state
    private Boolean hasLeftFront = false;
    private Boolean hasRightFront = false;
    private Boolean hasLeftRear = false;
    private Boolean hasRightRear = false;

    private DecimalFormat df = new DecimalFormat("###.####");
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        try {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_m");
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            hasLeftFront = true;
            telemetry.addData("Left Front", "Initialized");
        } catch (IllegalArgumentException iax) {
            telemetry.addData("Left Front", "Failed");
        }
        
        try {
            leftRearDrive  = hardwareMap.get(DcMotor.class, "left_rear_m");
            leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
            hasLeftRear = true;
            telemetry.addData("Left Rear", "Initialized");
        } catch (IllegalArgumentException iax) {
            telemetry.addData("Left Rear", "Failed");
        }
        
        try {
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_m");
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            hasRightFront = true;
            telemetry.addData("Right Front", "Initialized");
        } catch (IllegalArgumentException iax) {
            telemetry.addData("Right Front", "Failed");
        }

        try {
            rightRearDrive  = hardwareMap.get(DcMotor.class, "right_rear_m");
            rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
            hasRightRear = true;
            telemetry.addData("Right Rear", "Initialized");
        } catch (IllegalArgumentException iax) {
            telemetry.addData("Right Rear", "Failed");
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialization Complete");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (hasLeftFront){
            leftFrontDrive.setPower(0);
        }
        if (hasLeftRear){
            leftRearDrive.setPower(0);
        }
        if (hasRightFront){
            rightRearDrive.setPower(0);
        }
        if (hasLeftRear){
            rightRearDrive.setPower(0);
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftRearPower = 0;
        double rightRearPower = 0;

        // An initial x-crive data collection
        double driveX = gamepad1.left_stick_x;
        double driveY = -gamepad1.left_stick_y;
        double turn  = gamepad1.right_stick_x;
        double fastPower = gamepad1.left_trigger * 0.8;
        double slowPower = gamepad1.right_trigger * 0.2;
        driveX = driveX < 0.0000001 && driveX > -0.0000001 ? 0 : driveX;
        driveY = driveY < 0.0000001 && driveY > -0.0000001 ? 0 : driveY;
        
        // Calculate angle from controller direction
        double controllerAngle = (Math.abs(driveX) + Math.abs(driveY)) < 0.8 ? 0 :
                Math.toDegrees(Math.atan(driveX/driveY));
        // Transpose the rotation to foward = 0 and increasing right to positive 360
        controllerAngle = controllerAngle + (
                driveX > 0 && driveY > 0 ? 0 :
                driveY < 0 ? 180 :
                driveX < 0 ? 360 : 0);

        // Add the direction rate
        if ((Math.abs(driveX) + Math.abs(driveY)) < 0.8){
            leftFrontPower = 0;
            rightRearPower = 0;
            rightFrontPower = 0;
            leftRearPower = 0;
        } else if(controllerAngle >=0 && controllerAngle <= 90){
            leftFrontPower = 1;
            rightRearPower = 1;
            rightFrontPower = -Math.tan(Math.toRadians(controllerAngle-45));
            leftRearPower = -Math.tan(Math.toRadians(controllerAngle-45));
        } else if(controllerAngle >180 && controllerAngle <= 270){
            leftFrontPower = -1;
            rightRearPower = -1;
            rightFrontPower = Math.tan(Math.toRadians(controllerAngle-225));
            leftRearPower = Math.tan(Math.toRadians(controllerAngle-225));
        }else if(controllerAngle >90 && controllerAngle <= 180){
            rightFrontPower = -1;
            leftRearPower = -1;
            leftFrontPower = -Math.tan(Math.toRadians(controllerAngle-135));
            rightRearPower = -Math.tan(Math.toRadians(controllerAngle-135));
        } else if(controllerAngle >270 && controllerAngle <= 360){
            rightFrontPower = 1;
            leftRearPower = 1;
            leftFrontPower = -Math.tan(Math.toRadians(controllerAngle-315));
            rightRearPower = -Math.tan(Math.toRadians(controllerAngle-315));
        }

        // Scale the power by power buttons
        leftFrontPower = leftFrontPower * (fastPower + slowPower);
        leftRearPower = leftRearPower * (fastPower + slowPower);
        rightFrontPower = rightFrontPower * (fastPower + slowPower);
        rightRearPower = rightRearPower * (fastPower + slowPower);
        
        // Add the drive values from rotation
        leftFrontPower += turn * (fastPower + slowPower);
        leftRearPower += turn * (fastPower + slowPower);
        rightFrontPower += -turn * (fastPower + slowPower);
        rightRearPower += -turn * (fastPower + slowPower);

        // Normalize to max power of 1
        ArrayList<Double> powerList = new ArrayList<>();
        powerList.add(Math.abs(leftFrontPower));
        powerList.add(Math.abs(leftRearPower));
        powerList.add(Math.abs(rightFrontPower));
        powerList.add(Math.abs(rightRearPower));
        double maxPower = Collections.max(powerList);
        if (maxPower > 1.0) {
            leftFrontPower = leftRearPower / maxPower;
            leftRearPower = leftRearPower / maxPower;
            rightFrontPower = rightFrontPower / maxPower;
            rightRearPower = rightRearPower / maxPower;
        }

        if(hasLeftFront){
            leftFrontDrive.setPower(leftFrontPower);
        }
        if(hasRightFront){
            rightFrontDrive.setPower(rightFrontPower);
        }
        if(hasLeftRear){
            leftRearDrive.setPower(leftRearPower);
        }
        if(hasRightRear){
            rightRearDrive.setPower(rightRearPower);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Controller", " left Y(%.2f),     turn(%.2f)", driveY, turn);
        telemetry.addData("Controller", " left X(%.2f)", driveX);
        telemetry.addData("Direction", "  Angle (%.2f)", controllerAngle);
        telemetry.addData("Power", "      Fast  (%.2f),     Slow(%.2f)", fastPower, slowPower);
        telemetry.addData("Motor Power", " left front(%.2f), right front(%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Motor Power", " left rear (%.2f), right rear (%.2f)", leftRearPower, rightRearPower);
        telemetry.addData("Motors State", "left front(%b),   right front(%b)", hasLeftFront, hasRightFront);
        telemetry.addData("Motors State", "left rear (%b),   right rear (%b)", hasLeftRear, hasRightRear);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
