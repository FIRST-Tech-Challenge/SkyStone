/*
Copyright 2018 FIRST Tech Challenge Team [Phone] SAMSUNG

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */

@TeleOp(name="BubbleBot", group="Linear Opmode")
// @Disabled

public class BubbleBot extends LinearOpMode {


    private BNO055IMU imu;
    private DcMotor frontMotor;
    private DcMotor rightMotor;
    private DcMotor backMotor;
    private DcMotor leftMotor;
    private Blinker expansion_Hub_2;
    Orientation angles;
    Acceleration gravity;
    
    Orientation             lastAngles = new Orientation();
    
    double globalAngle = 0.3;



    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        frontMotor = hardwareMap.get(DcMotor.class, "frontMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        backMotor = hardwareMap.get(DcMotor.class, "backMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
//        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        
        imu.initialize(parameters);

        resetAngle();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        
        double LMP = 0;
        double RMP = 0;
        double FMP = 0;
        double BMP = 0;
        
        double FX = 0;
        double FY = 0;
        
        double CA = 0;
        
        double RSA = 0;
        
        double Mag = 0;
        
        double mult = 1;
        
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            
            sleep(50);
            idle();
        }
        
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            if (gamepad1.a) {
                resetAngle();
                
            }
            
            
            CA = (Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) * 180 / 3.14) + 45;
            
            RSA = CA * 3.14 / 180 - getAngle() * 3.14 / 180;
            
            Mag = Math.sqrt((gamepad1.right_stick_y * gamepad1.right_stick_y) + (gamepad1.right_stick_x * gamepad1.right_stick_x));
            
            FX = -Math.sin(RSA) * Mag;
            FY = -Math.cos(RSA) * Mag;
            
            telemetry.addData("X", FX);
            telemetry.addData("Y", FY);
            telemetry.addData("CA", CA);
            telemetry.addData("RSA", RSA);
            telemetry.addData("RA", getAngle());
            
            LMP = -gamepad1.left_stick_x + FX;
            RMP = -gamepad1.left_stick_x + -FX;
            FMP = -gamepad1.left_stick_x + FY;
            BMP = gamepad1.left_stick_x + FY;

            rightMotor.setPower(RMP * mult);
            leftMotor.setPower(LMP * mult);

            frontMotor.setPower(FMP * mult);
            backMotor.setPower(-BMP * mult);
            
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
    
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    

    //Function to get the angle of the Gyro sensor
    private double getAngle() {

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
