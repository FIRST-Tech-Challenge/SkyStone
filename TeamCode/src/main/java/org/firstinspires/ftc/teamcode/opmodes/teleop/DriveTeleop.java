package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.components.DriveSystem;


@TeleOp(name = "Drive", group="TeleOp")
public class DriveTeleop extends LinearOpMode {
    private DriveSystem driveSystem;
    private boolean slowDrive;

    public void initialize()
    {

        this.driveSystem.motorFrontLeft = hardwareMap.dcMotor.get("motorFL");
        this.driveSystem.motorFrontRight = hardwareMap.dcMotor.get("motorFR");
        this.driveSystem.motorBackRight = hardwareMap.dcMotor.get("motorBR");
        this.driveSystem.motorBackLeft = hardwareMap.dcMotor.get("motorBL");
        this.driveSystem.imuSystem.imu = hardwareMap.get(BNO055IMU.class, "imu");
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
