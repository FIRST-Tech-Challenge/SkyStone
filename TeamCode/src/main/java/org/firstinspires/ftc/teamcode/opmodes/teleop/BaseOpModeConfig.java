package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.DriveSystem;

import java.util.EnumMap;

public abstract class BaseOpModeConfig extends OpMode {
    private DriveSystem driveSystem;
    public ElapsedTime elapsedTime;   // Time into round.

    public void init(){
        telemetry.addLine("in init");

        EnumMap<DriveSystem.MotorNames, DcMotor> driveMap = new EnumMap<>(DriveSystem.MotorNames.class);
        for(DriveSystem.MotorNames name : DriveSystem.MotorNames.values()){
            driveMap.put(name,hardwareMap.get(DcMotor.class, name.toString()));
        }

        driveSystem = new DriveSystem(driveMap, hardwareMap.get(BNO055IMU.class, "imu"));


        elapsedTime = new ElapsedTime();
    }
}
