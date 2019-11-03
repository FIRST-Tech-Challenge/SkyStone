package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "ImuTest", group="Autonomous")
public class ImuTestOpMode extends OpMode {

    protected IMUSystem imu;
    private static final String TAG = "ImuTestOpMode";

    public void init(){

        imu = new IMUSystem(hardwareMap.get(BNO055IMU.class, "imu"));


    }

    public void loop(){
        Log.d(TAG,"heading:"  + imu.getHeading());
    }
}
