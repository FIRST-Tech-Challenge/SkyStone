package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.components.DriveSystem;

import java.util.EnumMap;


@TeleOp(name = "Drive", group="TeleOp")
public class DriveTeleop extends OpMode {

    private DriveSystem driveSystem;

    public void init(){
        telemetry.addLine("in init");

        EnumMap<DriveSystem.MotorNames, DcMotor> driveMap = new EnumMap<>(DriveSystem.MotorNames.class);
        for(DriveSystem.MotorNames name : DriveSystem.MotorNames.values()){
            telemetry.addLine("adding motors");
            driveMap.put(name,hardwareMap.get(DcMotor.class, name.toString()));
        }
        telemetry.update();

        driveSystem = new DriveSystem(driveMap, hardwareMap.get(BNO055IMU.class, "imu"));
    }


    public void loop(){
            float rx = gamepad1.right_stick_x;
            float ry = gamepad1.right_stick_y;
            float lx = gamepad1.left_stick_x;
            float ly = gamepad1.left_stick_y;

            if (gamepad1.a) {
                driveSystem.driveToPositionInches(50, DriveSystem.Direction.RIGHT, 0.5);
            }
            if (gamepad1.b) {
                driveSystem.driveToPositionInches(50, DriveSystem.Direction.LEFT, 0.5);
            }
            if (gamepad1.x) {
                driveSystem.driveToPositionInches(50, DriveSystem.Direction.FORWARD, 0.5);
            }
            if (gamepad1.y) {
                driveSystem.driveToPositionInches(50, DriveSystem.Direction.BACKWARD, 0.5);
            }
            driveSystem.drive(rx, ry, lx, ly);
    }
}
