package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.Vuforia;

import java.util.EnumMap;

public abstract class BaseOpModeConfig extends OpMode {
    protected DriveSystem driveSystem;
    protected Vuforia.CameraChoice currentCamera;

    protected Vuforia vuforia;

    public void init(){

        EnumMap<DriveSystem.MotorNames, DcMotor> driveMap = new EnumMap<>(DriveSystem.MotorNames.class);
        for(DriveSystem.MotorNames name : DriveSystem.MotorNames.values()){
            driveMap.put(name,hardwareMap.get(DcMotor.class, name.toString()));
        }
        driveSystem = new DriveSystem(driveMap, hardwareMap.get(BNO055IMU.class, "imu"));

        currentCamera = Vuforia.CameraChoice.PHONE_BACK;
        vuforia = new Vuforia(hardwareMap, currentCamera);
    }
}
