package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp (name = "GyroTest", group = "Test")
public class gyroTest extends LinearOpMode {

    GyroSensor gyro;

    @Override
    public void runOpMode() throws InterruptedException {

        gyro = hardwareMap.gyroSensor.get("gyro");

        waitForStart();
        while(opModeIsActive()) {
            gyro.calibrate();
            while (!gyro.isCalibrating()&&gamepad1.a){
                telemetry.addData("Updating?", gyro.isCalibrating());
                telemetry.update();
            }
            /*telemetry.addData("Updating?", gyro.isCalibrating());
            telemetry.update();*/
        }
    }
}
