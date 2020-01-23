package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassisGyro;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;
import org.firstinspires.ftc.teamcode.Library.OrientationTools;

@Autonomous (name = "Gyro_Time_Test")
public class GyroTimeTest extends LinearOpMode {
    HardwareChassis robot;
    OrientationTools orientationTools;
    HardwareChassisGyro robotGyro;
    OmniWheel omniWheel;

    @Override
    public void runOpMode() {

        orientationTools = new OrientationTools(robot);
        robot = new HardwareChassis(hardwareMap);
        robotGyro = new HardwareChassisGyro(hardwareMap);
        omniWheel = new OmniWheel(robot);

        waitForStart();

        if (opModeIsActive()) {
            orientationTools.driveSidewardTime(3000, 0.2, 500, robotGyro.imu, omniWheel, this);
        }
    }
}
