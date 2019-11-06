package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.monitor.MonitorIMU;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;

import java.util.Locale;

@Autonomous(name="Camera: help me", group="Linear Opmode")
public class CameraTestAuto extends AutoOpMode {
    @Override
    public void setup(DeviceMap map) {
        map.setUpVuforia(hardwareMap);
    }

    @Override
    public void beforeLoop() {


    }

    @Override
    public void run() {

    }
}
