package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareGyro;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;
import org.firstinspires.ftc.teamcode.Library.OrientationTools;


@TeleOp(name = "testing_Degree")
public class degree_testing extends OpMode {
    HardwareChassis hwmp;
    HardwareGyro gyro;
    OmniWheel wheel;
    OrientationTools tool;

    @Override
    public void init() {
        hwmp = new HardwareChassis(hardwareMap);
        wheel = new OmniWheel(hwmp);
        gyro = new HardwareGyro(hardwareMap);
        tool = new OrientationTools(hwmp);
        gyro.init(hardwareMap);
        //this.msStuckDetectLoop = 1073741824;
    }

    @Override
    public void loop() {

        if(gamepad1.a)
            tool.driveSidewardTime((long)(2000),0.2,(double)(100),gyro.imu,this.wheel,this);
        if(gamepad1.b)
            tool.driveSidewardEncoderV2(20,0.2,100,gyro.imu,this.wheel,this,this.tool.getDegree360(gyro.imu));
        if(gamepad1.y)
            tool.simeoncopytry(0,20,0.2,this.wheel,this.tool.getDegree360(this.gyro.imu),this.gyro.imu,100);

    }
}

