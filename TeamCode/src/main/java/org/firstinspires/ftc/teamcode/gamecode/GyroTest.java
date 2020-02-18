package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

@Autonomous

public class GyroTest extends AutoOpMode {
    private VoltageSensor ExpansionHub2_VoltageSensor;
    public void runOp() throws InterruptedException {
        Joules joules = new Joules();
        ColorSensor colorSensor;
        GyroSensor gyroSensor;


        ExpansionHub2_VoltageSensor =  hardwareMap.voltageSensor.get("Expansion Hub 2");

        colorSensor = hardwareMap.colorSensor.get("colour");
//        gyroSensor = hardwareMap.gyroSensor.get("imu");

        telemetry.addData("Status", "initialized");

        int bluetapeval = 27;

        waitForStart();

//        while(opModeIsActive()){
//            telemetry.addData("Rotation Factor", gyroSensor.getRotationFraction());
//            telemetry.addData("is calibrating", gyroSensor.isCalibrating());
//            telemetry.addData("RawX", gyroSensor.rawX());
//            telemetry.addData("RawY", gyroSensor.rawY());
//            telemetry.addData("RawZ", gyroSensor.rawZ());
//        }





    }

}
