package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Mecanum teleop (with an optional arcade mode)
 * * Left stick controls x/y translation.
 * * Right stick controls rotation about the z axis
 * * When arcade mode is enabled (press "a"), translation direction
 * becomes relative to the field as opposed to the robot. You can
 * reset the forward heading by pressing "x".
 */
@TeleOp(name = "Manual Drive Blue")
public class ManualDriveOpModeBlueWall extends LinearOpMode {
    private servoDropBot robot = new servoDropBot(this);
    private boolean arcadeMode = false;
    private int gyroCalibratedCount = 0;



    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {

            robot.driveByHandBlueWall(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            robot.scoopSetPosition(gamepad1.b, gamepad1.y, gamepad1.x);
            robot.scoopFreeRun(gamepad1.right_bumper, gamepad1.left_bumper);
            robot.manualFoundationDrop(gamepad1.dpad_down);
            robot.manualFoundationReset(gamepad1.dpad_up);
            robot.manualPickupSkystone(gamepad1.dpad_left);
            robot.manualDropSkystone(gamepad1.dpad_right);
            robot.toggleButtArm(gamepad1.right_stick_button, gamepad1.left_stick_button);
            robot.toggleServoDrop(gamepad2.a);
        }
    }
}

