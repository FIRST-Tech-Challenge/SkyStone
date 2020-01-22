package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareGyro;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;
import org.firstinspires.ftc.teamcode.Library.OrientationTools;

//by Lena and Simeon

@TeleOp(name = "GyroPlayerTest")

public class GyroPlayerTest extends OpMode {
    HardwareGyro gyro;
    OmniWheel oWheel;
    HardwareChassis robot;
    ColorTools colorTools;
    OrientationTools oTool;

    @Override
    public void init() {
        robot = new HardwareChassis(hardwareMap);
        colorTools = new ColorTools();
        oWheel = new OmniWheel(robot);
        oTool = new OrientationTools(robot);


    }

    @Override
    public void loop() {



    }

    public void driveSidewardGamepad(Gamepad g, double power, double smoothness, BNO055IMU imu, OmniWheel wheel, OpMode op, OrientationTools oTools){
        double offset = oTools.getDegree() - Math.atan(g.left_stick_y/g.left_stick_x);
        double startPos = oTools.getDegree() - offset;
        while(g.left_stick_x !=0 || g.left_stick_y !=0){
            wheel.setMotors(g.left_stick_x,g.left_stick_y,(startPos-(oTools.getDegree()-offset))/smoothness);
        }
        wheel.setMotors(0,0,0);
    }

}


