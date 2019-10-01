package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.DriveSystem;


@TeleOp(name = "Drive", group="TeleOp")
public class DriveTeleop extends LinearOpMode {
    private DriveSystem driveSystem;
    private boolean slowDrive;

    public void initialize() {

        DcMotor[] motors = {hardwareMap.dcMotor.get("motorFL"), hardwareMap.dcMotor.get("motorFR"),
                            hardwareMap.dcMotor.get("motorBR"), hardwareMap.dcMotor.get("motorBL") };
        this.driveSystem = new DriveSystem(motors, hardwareMap.get(BNO055IMU.class, "imu"));
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
