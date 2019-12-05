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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

@Autonomous(name = "RED_BLOCK", group = "Linear Opmode")
public class RED_Platform_BridgeGYRO_BLOCK_INC extends BaseAutoOpMode {


    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = 1, correction;

    int startingSide = -1;  //Set to 1 for blue and -1 for Red


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        GetHardware();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Robot is strafes right then stops
        front_left.setPower(1 * startingSide);
        rear_left.setPower(-.5 * startingSide);
        front_right.setPower(-.5 * startingSide);
        rear_right.setPower(.5 * startingSide);
        //sleep(250);
        sleep(725);
        CutMotors();

        // Larson added this. This is added to have the robot move back to the wall to hopefully straighten it.
        RunAllMotorsBackward();
        sleep(100);

        UnfoldRobot();

        //Clamps half
        //Clamp_Left.setPosition(0.5);
        //Clamp_Right.setPosition(0.5);
        //sleep(1000);

        //Robot Drives Forward
        //RunAllMotors();
        //sleep(225);
        //CutMotors();

        //Clamps are down
        Clamp_Left.setPosition(1);
        Clamp_Right.setPosition(0f);
        sleep(1500);

        //Robot moves backwards
        RunAllMotorsBackward();
        sleep(600);


        //Robot strafes right
        front_left.setPower(-1 * startingSide);
        rear_left.setPower(.7 * startingSide);
        front_right.setPower(1 * startingSide);
        rear_right.setPower(-1 * startingSide);
        sleep(1200);
        CutMotors();
        //sleep(600);

        //front_left.setPower(1);
        //rear_left.setPower(1);
        //front_right.setPower(-1);
        //rear_right.setPower(-1);
        //sleep(1900);

        //Turns right
        rotate(55 * startingSide, 1);

        //Robot drives platform
        RunAllMotors();
        sleep(600);
        CutMotors();

        //End of moving platform

        //Clamps go up
        Clamp_Left.setPosition(0f);
        Clamp_Right.setPosition(1f);
        Release_Servo.setPosition(0.4);
        // sleep(500);

        //Pulley system moves backwards
        top_motor.setPower(-1);
        sleep(200);

        //Control system that stops pulley
        while (Top_Sensor_Rear.getState()) {
            top_motor.setPower(0.7);
            telemetry.addData("Loop Crane ", Top_Sensor_Rear.getState());
            telemetry.update();
        }

        top_motor.setPower(0);

        //Control system that gets pulley system to go
        while (bottom_touch.getState()) {
            lift_left.setPower(1);
            lift_right.setPower(1);
            telemetry.addData("Loop Lift ", bottom_touch.getState());
            telemetry.update();
        }
        telemetry.addData("Loop Lift ", "Out Of Loop");
        telemetry.update();

        //top_motor.setPower(0);


        //  lift_left.setPower(1);
        //  lift_right.setPower(1);
        //  sleep(1500);

        //Lift
        lift_left.setPower(0);
        lift_right.setPower(0);

        //Open Claw
        Block_Pickup.setPosition(0.4);
        sleep(1000);


        RunAllMotorsBackward();
        sleep(1400);
        CutMotors();

        //reset gyro and rotate 30
        feeder_motor.setPower(-1);
        resetAngle();
        rotate(-30, 1);


        //turn on feeder and drive backwards
        feeder_motor.setPower(-1);
        RunAllMotorsBackward();
        sleep(625);
        CutMotors();

        //keep feeder on
        feeder_motor.setPower(-1);

        //Drive Forward
        RunAllMotors();
        sleep(650);
        CutMotors();

        //rotate back
        resetAngle();
        rotate(40, 1);


        //Drive Forward
        RunAllMotors();
        sleep(1350);
        CutMotors();

        //Close Claw
        Block_Pickup.setPosition(1);
        sleep(1000);

        //turn off feeder
        feeder_motor.setPower(0);
        //Crane Up
        lift_left.setPower(-1);
        lift_right.setPower(-1);
        sleep(1000);

        //Lift Stop
        lift_left.setPower(0);
        lift_right.setPower(0);

        //Crane Across
        top_motor.setPower(-1);
        while (Top_Sensor_Front.getState()) {
            top_motor.setPower(-1);
            telemetry.addData("Status", "Moving Crane");
            telemetry.update();
        }
        top_motor.setPower(0);

        //Open Claw
        Block_Pickup.setPosition(0.4);
        sleep(1000);

        //Crane Across
        top_motor.setPower(-1);
        while (Top_Sensor_Rear.getState()) {
            top_motor.setPower(1);
            telemetry.addData("Status", "Moving Crane");
            telemetry.update();
        }
        top_motor.setPower(0);

        //Lift Down
        lift_left.setPower(1);
        lift_right.setPower(1);
        sleep(900);

        RunAllMotorsBackward();
        sleep(550);


    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        front_left.setPower(leftPower);
        rear_left.setPower(leftPower);
        front_right.setPower(rightPower);
        rear_right.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        front_left.setPower(0);
        rear_left.setPower(0);
        front_right.setPower(0);
        rear_right.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

}

