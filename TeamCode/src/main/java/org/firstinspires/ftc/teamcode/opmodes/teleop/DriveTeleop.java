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
<<<<<<< refs/remotes/origin/Arm

<<<<<<< refs/remotes/origin/Arm
<<<<<<< refs/remotes/origin/Arm
=======
=======
>>>>>>> Removed slowdrive. Reintroduced Direction enum and isStrafe method to DriveSystem.
        telemetry.update();

>>>>>>> added some stuff to op mode to make it work.
        driveSystem = new DriveSystem(driveMap, hardwareMap.get(BNO055IMU.class, "imu"));
<<<<<<< refs/remotes/origin/Arm
=======
        this.driveSystem = new DriveSystem(this);
>>>>>>> Revert "fixed a compile error in Driveteleop.java"
        slowDrive = false;
=======
>>>>>>> Removed slowdrive. Reintroduced Direction enum and isStrafe method to DriveSystem.
    }


    public void loop(){
            float rx = gamepad1.right_stick_x;
            float ry = gamepad1.right_stick_y;
            float lx = gamepad1.left_stick_x;
            float ly = gamepad1.left_stick_y;
            driveSystem.drive(rx, ry, lx, ly);
    }
}
