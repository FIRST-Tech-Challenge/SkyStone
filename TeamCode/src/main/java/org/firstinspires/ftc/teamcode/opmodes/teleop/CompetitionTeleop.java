package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.ArmSystem;
import org.firstinspires.ftc.teamcode.components.DriveSystem.Direction;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

import java.util.EnumMap;

@TeleOp(name = "CompetitionTeleop", group="TeleOp")
public class CompetitionTeleop extends BaseOpMode {

    ArmSystem armSystem;
    

    public void init() {
        // Not sure if I'm doing EnumMaps 100% right so please lmk if I'm not so I can change it
        EnumMap<ArmSystem.ServoNames, Servo> servoEnumMap = new EnumMap<ArmSystem.ServoNames, Servo>(ArmSystem.ServoNames.class);
        servoEnumMap.put(ArmSystem.ServoNames.GRIPPER, hardwareMap.get(Servo.class, "gripper"));
        servoEnumMap.put(ArmSystem.ServoNames.ELBOW, hardwareMap.get(Servo.class, "elbow"));
        servoEnumMap.put(ArmSystem.ServoNames.WRIST, hardwareMap.get(Servo.class, "wrist"));
        servoEnumMap.put(ArmSystem.ServoNames.PIVOT, hardwareMap.get(Servo.class, "pivot"));
        armSystem = new ArmSystem(
                servoEnumMap,
                hardwareMap.get(DcMotor.class, "slider_motor"),
                hardwareMap.get(DigitalChannel.class, "slider_switch"));


    }

    public void loop(){

        // Put every joystick value to the 3rd power for greater control over the robot
        // 1^3 = 1, so we don't even need to trim the values or anything
        float rx = (float) Math.pow(gamepad1.right_stick_x, 3);
        float lx = (float) Math.pow(gamepad1.left_stick_x, 3);
        float ly = (float) Math.pow(gamepad1.left_stick_y, 3);

        if (gamepad1.a) {
            driveSystem.driveToPosition(1000, Direction.RIGHT, 0.5);
        }
        if (gamepad1.b) {
            driveSystem.driveToPosition(1000, Direction.LEFT, 0.5);
        }
        if (gamepad1.x) {
            driveSystem.driveToPosition(1000, Direction.FORWARD, 0.5);
        }
        if (gamepad1.y) {
            driveSystem.driveToPosition(1000, Direction.BACKWARD, 0.5);
        }
        driveSystem.drive(rx, lx, ly);
    }
}