package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareGyro;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;
import org.firstinspires.ftc.teamcode.Library.OrientationTools;


@Disabled
@TeleOp(name = "turn test")
public class testing_turning extends LinearOpMode {
    HardwareGyro gyro;
    OmniWheel oWheel;
    HardwareChassis robot;
    ColorTools colorTools;
    OrientationTools oTool;

    double smoothness;
    double offset;
    double posTodrive;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareChassis(hardwareMap);
        colorTools = new ColorTools();
        oWheel = new OmniWheel(robot);
        oTool = new OrientationTools(robot,hardwareMap,null);
        gyro = new HardwareGyro(hardwareMap);
        gyro.init(hardwareMap);


        oTool.turnToDegrees(90,200,oWheel,gyro.imu);

    }
}
