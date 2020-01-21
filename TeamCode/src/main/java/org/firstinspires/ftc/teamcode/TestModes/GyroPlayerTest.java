package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareGyro;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;

//by Lena and Simeon

@TeleOp(name = "GyroPlayerTest")

public class GyroPlayerTest extends OpMode {
    HardwareGyro gyro;
    OmniWheel oWheel;
    HardwareChassis robot;
    ColorTools colorTools;
    double smootingValue;
    double[] liftZeros = new double[2];

    @Override
    public void init() {
        robot = new HardwareChassis(hardwareMap);
        colorTools = new ColorTools();
        oWheel = new OmniWheel(robot);



    }

    @Override
    public void loop() {



    }

}


