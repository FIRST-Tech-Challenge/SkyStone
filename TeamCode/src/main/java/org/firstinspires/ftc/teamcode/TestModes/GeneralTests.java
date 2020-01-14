package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareGyro;
@TeleOp(name="tests")
public class GeneralTests extends OpMode {
    HardwareGyro gyro;
    @Override
    public void init() {
        gyro = new HardwareGyro(hardwareMap);
        gyro.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("X°",gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("Y°",gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
        telemetry.addData("Z°",gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.update();
    }
}
