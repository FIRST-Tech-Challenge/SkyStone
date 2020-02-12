package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
public class degree_testing extends LinearOpMode {
    HardwareChassis hwmp;
    HardwareGyro gyro;
    OmniWheel wheel;
    OrientationTools tool;

    double startPos;


    @Override
    public void runOpMode() {
        hwmp = new HardwareChassis(hardwareMap);
        wheel = new OmniWheel(hwmp);
        gyro = new HardwareGyro(hardwareMap);
        tool = new OrientationTools(hwmp, hardwareMap, this);
        gyro.init(hardwareMap);
        startPos = tool.getDegree360(gyro.imu);

        waitForStart();

        if (opModeIsActive()) {
            tool.driveSidewardEncoder(this, 0, -300, -0.3, wheel, startPos, gyro.imu, 200, 125);
        }

    }
}

