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
    }

    @Override
    public void loop() {
        telemetry.addData("°",gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
        tool.driveSidewardEncoder(new int[]{0,0,0,0},0.1,100,gyro.imu,wheel,this);

    }
}
