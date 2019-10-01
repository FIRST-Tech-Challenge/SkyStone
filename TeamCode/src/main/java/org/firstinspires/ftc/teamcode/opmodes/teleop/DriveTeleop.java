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
public class DriveTeleop extends LinearOpMode {

    private DriveSystem driveSystem;
    private boolean slowDrive;

    public void initialize(){
        EnumMap<DriveSystem.MotorNames, DcMotor> driveMap = new EnumMap<>(DriveSystem.MotorNames.class);
        for(DriveSystem.MotorNames name : DriveSystem.MotorNames.values()){
            driveMap.put(name,hardwareMap.get(DcMotor.class, name.toString());
        }

<<<<<<< refs/remotes/origin/Arm
        driveSystem = new DriveSystem(driveMap, hardwareMap.get(BNO055IMU.class, "imu"));
=======
        this.driveSystem = new DriveSystem(this);
>>>>>>> Revert "fixed a compile error in Driveteleop.java"
        slowDrive = false;
    }


    public void runOpMode(){
        initialize();

        float rx = gamepad1.right_stick_x;
        float ry = gamepad1.right_stick_y;
        float lx = gamepad1.left_stick_x;
        float ly = gamepad1.left_stick_y;

        driveSystem.drive(rx, ry, lx, ly, slowDrive);
    }

}
