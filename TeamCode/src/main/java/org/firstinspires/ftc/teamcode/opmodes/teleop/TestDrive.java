package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

import java.util.EnumMap;

@TeleOp(name = "TeleopTestDrive", group="TeleOp")
public class TestDrive extends OpMode {
    protected DriveSystem driveSystem;
    public void init() {
        EnumMap<DriveSystem.MotorNames, DcMotor> driveMap = new EnumMap<>(DriveSystem.MotorNames.class);
        for(DriveSystem.MotorNames name : DriveSystem.MotorNames.values()){
            driveMap.put(name,hardwareMap.get(DcMotor.class, name.toString()));
        }

        driveSystem = new DriveSystem(driveMap, hardwareMap.get(BNO055IMU.class, "imu"));
    }
    public void loop() {
//        float rx = (float) Math.pow(gamepad1.right_stick_x, 5);
//        float lx = (float) Math.pow(gamepad1.left_stick_x, 5);
//        float ly = (float) Math.pow(gamepad1.left_stick_y, 5);
        float rx = gamepad1.right_stick_x;
        float lx = gamepad1.left_stick_x;
        float ly = gamepad1.left_stick_y;
        telemetry.addData("TestDrive","left trig -- " + gamepad1.left_trigger);
        Log.d("TestDrive", "left trig -- " + gamepad1.left_trigger);
        driveSystem.drive(rx, lx, -ly, gamepad1.left_trigger);
    }
}
