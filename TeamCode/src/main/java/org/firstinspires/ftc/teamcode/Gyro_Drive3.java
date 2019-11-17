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
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */

@Autonomous(name = "Gyro_Drive3", group = "Linear Opmode")
public class Gyro_Drive3 extends BaseAutoOpMode {

    BNO055IMU imu;
    float globalAngle = 0;
    Orientation lastAngles = new Orientation();

    float MotorPower = 0.6f;



    @Override
    public void runOpMode() {


        InitializeIMU();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        GetHardware();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ResetAngle();


        while (opModeIsActive()) {



            front_left.setPower(1);
            rear_left.setPower(-MotorPower);
            front_right.setPower(-1);
            rear_right.setPower(MotorPower);

            if(gamepad1.a){
                MotorPower = MotorPower + 0.1f;
            }
            else if (gamepad1.b){
                MotorPower = MotorPower - 0.1f;
            }

            telemetry.addData("Motor Power", MotorPower);


//            if(GetAngle() > 5){
//
//                telemetry.addData("Tripped", " + 5 ");
//                front_left.setPower(0.4);
//                rear_left.setPower(-0.4);
//                front_right.setPower(-1);
//                rear_right.setPower(1);
//            }
//            else if (GetAngle() < -5){
//
//                telemetry.addData("Tripped", " - -5 ");
//                front_left.setPower(1);
//                rear_left.setPower(-1);
//                front_right.setPower(-0.4);
//                rear_right.setPower(0.4);
//            }
//            else{
//
//                front_left.setPower(1);
//                rear_left.setPower(-1);
//                front_right.setPower(-1);
//                rear_right.setPower(1);
//
//            }
            telemetry.addData("Angle" , GetAngle());
            telemetry.update();
        }

    }
    //Set up the IMU
    public void InitializeIMU ()
    {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //Set the current angle to 0
    public void ResetAngle ()
    {
        //Read the current angle from the IMU
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //Set the current angle to 0
        globalAngle = 0;

    }
    //Get the current angle
    public float GetAngle ()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        //Request the angle from the Sensor
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //Figure out the difference since the last angle and the current angle
        float deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        //Convert Euler angles
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        //Add the difference to the Global Angle
        globalAngle += deltaAngle;

        //Store the current angle as "lastAngles" - this sets us up for the next time the function is called
        lastAngles = angles;

        return globalAngle;

    }
}


