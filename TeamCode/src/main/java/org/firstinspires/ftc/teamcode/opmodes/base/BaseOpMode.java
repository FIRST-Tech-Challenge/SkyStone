package org.firstinspires.ftc.teamcode.opmodes.base;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.IMUSystem;
import org.firstinspires.ftc.teamcode.components.Vuforia;
import org.firstinspires.ftc.teamcode.components.Vuforia.CameraChoice;

import java.util.EnumMap;

public abstract class BaseOpMode extends OpMode {

    protected DriveSystem driveSystem;
    protected Vuforia vuforia;
    protected VuforiaTrackable skystone;
    protected VuforiaTrackable rearPerimeter;

    public void init(){
        this.msStuckDetectInit = 20000;
        this.msStuckDetectInitLoop = 20000;
        EnumMap<DriveSystem.MotorNames, DcMotor> driveMap = new EnumMap<>(DriveSystem.MotorNames.class);
        for(DriveSystem.MotorNames name : DriveSystem.MotorNames.values()){
            driveMap.put(name,hardwareMap.get(DcMotor.class, name.toString()));
        }
        driveSystem = new DriveSystem(driveMap, hardwareMap.get(BNO055IMU.class, "imu"));
    }

    protected void setCamera(CameraChoice cameraChoice){
        vuforia = new Vuforia(hardwareMap, cameraChoice);
        skystone = vuforia.targetsSkyStone.get(0);
    }
}
