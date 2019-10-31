package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.DriveSystem.Direction;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

@TeleOp(name = "Drive", group="TeleOp")
public class DriveTeleop extends BaseOpMode {

    public void loop(){
            float rx = gamepad1.right_stick_x;
            float lx = gamepad1.left_stick_x;
            float ly = gamepad1.left_stick_y;

            if (gamepad1.a) {
                driveSystem.motors.forEach((name, motor) -> {
                    switch(name) {
                        case FRONTLEFT:
                            motor.setPower(0.5);
                        case BACKLEFT:
                        case FRONTRIGHT:
                        case BACKRIGHT:
                            break;
                    }
                });
            }
            if (gamepad1.b) {
                driveSystem.motors.forEach((name, motor) -> {
                    switch(name) {
                        case FRONTLEFT:
                            break;
                        case BACKLEFT:
                            motor.setPower(0.5);
                        case FRONTRIGHT:
                        case BACKRIGHT:
                            break;
                    }
                });
            }
            if (gamepad1.x) {
                driveSystem.motors.forEach((name, motor) -> {
                    switch(name) {
                        case FRONTLEFT:
                        case BACKLEFT:
                            break;
                        case FRONTRIGHT:
                            motor.setPower(0.5);
                        case BACKRIGHT:
                            break;
                    }
                });
            }
            if (gamepad1.y) {
                driveSystem.motors.forEach((name, motor) -> {
                    switch(name) {
                        case FRONTLEFT:
                        case BACKLEFT:
                        case FRONTRIGHT:
                            break;
                        case BACKRIGHT:
                            motor.setPower(0.5);
                    }
                });
            }
            driveSystem.drive(rx, lx, ly);
    }
}