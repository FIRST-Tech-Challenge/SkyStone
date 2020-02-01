
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
@TeleOp(name = "Manual Drive")
public class ManualDriveOpMode extends LinearOpMode {
    private servoDropBot robot = new servoDropBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {

            robot.driveByHand(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            robot.scoopSetPosition(gamepad1.b, gamepad1.y, gamepad1.x);
            robot.manualOperateSkyStone(gamepad2.left_bumper, gamepad2.right_bumper);
            robot.manualDropSkystone(gamepad1.dpad_right);
            robot.manualArmRelease(gamepad2.b);
            robot.manualArmPinch(gamepad2.x);

            robot.toggleButtArm(gamepad2.right_stick_button, gamepad2.left_stick_button);
            robot.toggleServoDrop(gamepad2.y, gamepad2.a);

        }
    }
}