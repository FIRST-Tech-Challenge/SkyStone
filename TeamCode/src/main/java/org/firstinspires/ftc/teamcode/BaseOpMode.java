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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

public abstract class BaseOpMode extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor front_left = null;
    public DcMotor rear_left = null;
    public DcMotor front_right = null;
    public DcMotor rear_right = null;
    public DcMotor lift_left = null;
    public DcMotor lift_right = null;
    public DcMotor feeder_motor = null;
    public DcMotor top_motor = null;
    public Servo Clamp_Left = null;
    public Servo Clamp_Right = null;
    public Servo Feeder_Servo = null;
    public Servo Block_Pickup = null;
    public Servo Capstone = null;
    public Servo Release_Servo = null;
    //public Servo Release_Servo2 = null;
    public DigitalChannel Top_Sensor_Front = null;
    public DigitalChannel Top_Sensor_Rear = null;
    public DigitalChannel bottom_touch = null;
    public DigitalChannel top_touch = null;

    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public double globalAngle;
    int loop = 0;


    public void GetHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        lift_left = hardwareMap.get(DcMotor.class, "lift_left");
        lift_right = hardwareMap.get(DcMotor.class, "lift_right");
        feeder_motor = hardwareMap.get(DcMotor.class, "feeder_motor");
        top_motor = hardwareMap.get(DcMotor.class, "top_motor");
        Clamp_Left = hardwareMap.get(Servo.class, "Clamp_Left");
        Clamp_Right = hardwareMap.get(Servo.class, "Clamp_Right");
        Feeder_Servo = hardwareMap.get(Servo.class, "Feeder_Servo");
        Block_Pickup = hardwareMap.get(Servo.class, "Block_Pickup");
        Capstone = hardwareMap.get(Servo.class, "Capstone");
        Release_Servo = hardwareMap.get(Servo.class, "Release_Servo");
        //Release_Servo2 = hardwareMap.get(Servo.class, "Release_Servo2");
        Top_Sensor_Rear = hardwareMap.get(DigitalChannel.class, "Top_Sensor_Rear");
        Top_Sensor_Front = hardwareMap.get(DigitalChannel.class, "Top_Sensor_Front");
        bottom_touch = hardwareMap.get(DigitalChannel.class, "bottom_touch");
        top_touch = hardwareMap.get(DigitalChannel.class, "top_touch");


        // set digital channel to input mode.
        Top_Sensor_Front.setMode(DigitalChannel.Mode.INPUT);
        Top_Sensor_Rear.setMode(DigitalChannel.Mode.INPUT);
        bottom_touch.setMode(DigitalChannel.Mode.INPUT);
        top_touch.setMode(DigitalChannel.Mode.INPUT);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        front_left.setDirection(DcMotor.Direction.FORWARD);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.FORWARD);
        lift_left.setDirection(DcMotor.Direction.REVERSE);
        lift_right.setDirection(DcMotor.Direction.REVERSE);
        feeder_motor.setDirection(DcMotor.Direction.REVERSE);
        top_motor.setDirection(DcMotor.Direction.FORWARD);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        GetIMU();
    }


    public void GetIMU() {
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

        telemetry.addData("IMU", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("IMU", "calibrated");
        telemetry.update();
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power) {
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

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
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


    public enum DriveDirection {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        STOP,
        STRAFE_RIGHT,
        STRAFE_LEFT
    }


    public enum Mode {
        STOP_RESET_ENCODER,
        RUN_WITH_ENCODER,
        RUN_WITHOUT_ENCODERS,
    }


    public enum LiftDirection {
        STOP,
        UP,
        DOWN
    }


    public void StopAllDrive() {
        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);
    }


    public void Drive(DriveDirection direction) {
        if (direction == DriveDirection.STOP) {
            front_left.setPower(0);
            front_right.setPower(0);
            rear_left.setPower(0);
            rear_right.setPower(0);
        }
        if (direction == DriveDirection.RIGHT) {
            front_left.setPower(1);
            front_right.setPower(-1);
            rear_left.setPower(1);
            rear_right.setPower(-1);
        }
        if (direction == DriveDirection.LEFT) {
            front_left.setPower(-1);
            front_right.setPower(1);
            rear_left.setPower(-1);
            rear_right.setPower(1);
        }
        if (direction == DriveDirection.FORWARD) {
            front_left.setPower(1);
            front_right.setPower(1);
            rear_left.setPower(1);
            rear_right.setPower(1);
        }
        if (direction == DriveDirection.BACKWARD) {
            front_left.setPower(-1);
            front_right.setPower(-1);
            rear_left.setPower(-1);
            rear_right.setPower(-1);


        }
    }


    public void EncoderDrive(DriveDirection direction, int EncoderValue) {
        if (direction == DriveDirection.STOP) {
            front_left.setPower(0);
            front_right.setPower(0);
            rear_left.setPower(0);
            rear_right.setPower(0);

        }
        if (direction == DriveDirection.RIGHT) {

            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);


            while(front_left.getCurrentPosition() < EncoderValue) {
                front_left.setPower(1);
                front_right.setPower(-1);
                rear_left.setPower(1);
                rear_right.setPower(-1);
                sleep(1);
            }

            SetDriveMode(Mode.STOP_RESET_ENCODER);

            //front_left.setTargetPosition(EncoderValue
        }
        if (direction == DriveDirection.LEFT) {

            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);


            while(front_left.getCurrentPosition() < EncoderValue) {
                front_left.setPower(-1);
                front_right.setPower(1);
                rear_left.setPower(-1);
                rear_right.setPower(1);
                sleep(1);
            }

            SetDriveMode(Mode.STOP_RESET_ENCODER);


        }
        if (direction == DriveDirection.FORWARD) {

            //SetDriveMode(Mode.STOP_RESET_ENCODER);
            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);


            while(front_left.getCurrentPosition() < EncoderValue) {
                front_left.setPower(1);
                front_right.setPower(1);
                rear_left.setPower(1);
                rear_right.setPower(1);
                sleep(1);
            }

            SetDriveMode(Mode.STOP_RESET_ENCODER);
        }
        if (direction == DriveDirection.BACKWARD) {
            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);


            while(front_left.getCurrentPosition() > -EncoderValue) {
                front_left.setPower(-1);
                front_right.setPower(-1);
                rear_left.setPower(-1);
                rear_right.setPower(-1);
                sleep(1);

                telemetry.addData("Encoder", front_left.getCurrentPosition());
                telemetry.update();
            }

            SetDriveMode(Mode.STOP_RESET_ENCODER);

            //front_left.setTargetPosition(EncoderValue
        }

        if (direction == DriveDirection.STRAFE_LEFT) {
            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);


            while(front_left.getCurrentPosition() > -EncoderValue) {
                front_left.setPower(-1);
                front_right.setPower(1);
                rear_left.setPower(1);
                rear_right.setPower(-1);
                sleep(1);

                telemetry.addData("Encoder", front_left.getCurrentPosition());
                telemetry.update();
            }

            SetDriveMode(Mode.STOP_RESET_ENCODER);



        }
        if (direction == DriveDirection.STRAFE_RIGHT) {
            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);


            while(front_left.getCurrentPosition() < EncoderValue) {
                front_left.setPower(1);
                front_right.setPower(-1);
                rear_left.setPower(-1);
                rear_right.setPower(1);
                sleep(1);

                telemetry.addData("Encoder", front_left.getCurrentPosition());
                telemetry.update();
            }

            SetDriveMode(Mode.STOP_RESET_ENCODER);


        }


    }


    public void Strafe(DriveDirection direction) {
        if (direction == DriveDirection.LEFT) {
            front_left.setPower(-0.8);
            front_right.setPower(0.7);
            rear_left.setPower(0.8);
            rear_right.setPower(-0.8);
        }
        if (direction == DriveDirection.RIGHT) {
            front_left.setPower(1);
            front_right.setPower(-0.5);
            rear_left.setPower(-0.5);
            rear_right.setPower(0.5);
        }
    }


    public void Lift(LiftDirection direction) {
        if (direction == LiftDirection.STOP) {
            lift_left.setPower(0);
            lift_right.setPower(0);
        }
        if (direction == LiftDirection.UP) {
            lift_left.setPower(1);
            lift_right.setPower(1);
        }
        if (direction == LiftDirection.DOWN) {
            lift_left.setPower(-1);
            lift_right.setPower(-1);
        }

    }


    public void SetDriveMode(Mode DriveMode) {

        if (DriveMode == Mode.STOP_RESET_ENCODER) {

            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
        if (DriveMode == Mode.RUN_WITH_ENCODER) {

            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        if (DriveMode == Mode.RUN_WITHOUT_ENCODERS) {

            front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

    }
}






