package org.firstinspires.ftc.teamcode.opmodes.teleop;


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

        this.driveSystem = new DriveSystem(this);
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
