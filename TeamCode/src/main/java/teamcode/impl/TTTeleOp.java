package teamcode.impl;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.common.TTDriveSystem;
import teamcode.common.TTHardwareManager;
import teamcode.common.TTOpMode;
import teamcode.common.TTRobot;
import teamcode.common.Vector2;

@TeleOp(name = "TT TeleOp")
public class TTTeleOp extends TTOpMode {

    private static final double REDUCED_DRIVE_SPEED_MULTIPLIER = 0.6;
    private static final double TURN_SPEED_MULTIPLIER = 0.6;

    private TTDriveSystem driveSystem;

    @Override
    protected void onInitialize() {
        setHardwareRestrictions(TTHardwareManager.TTHardwareRestriction.DISABLE_INTAKE);
    }

    @Override
    protected void onStart() {
        TTRobot robot = getRobot();
        driveSystem = robot.getHardwareManager().getDriveSystem();

        while (opModeIsActive()) {
            update();
            //driveSystem.continuous(Vector2.FORWARD, 0);
        }
    }

    private void update() {
        driveUpdate();
    }

    private void driveUpdate() {
        double vertical = gamepad1.right_stick_y;
        double horizontal = gamepad1.right_stick_x;

        Vector2 velocity;
        if (Math.abs(vertical) > Math.abs(horizontal)) {
            // vertical movement takes precedence
            if (vertical > 0) {
                velocity = Vector2.FORWARD.multiply(vertical);
            } else {
                velocity = Vector2.BACKWARD.multiply(-vertical);
            }
        } else if (Math.abs(vertical) < Math.abs(horizontal)) {
            // horizontal movement takes precedence
            if (horizontal > 0) {
                velocity = Vector2.RIGHT.multiply(horizontal);
            } else {
                velocity = Vector2.LEFT.multiply(-horizontal);
            }
        } else {
            velocity = Vector2.ZERO;
        }

        if (!gamepad1.right_bumper) {
            velocity = velocity.multiply(REDUCED_DRIVE_SPEED_MULTIPLIER);
        }

        double turn = gamepad1.left_stick_x * TURN_SPEED_MULTIPLIER;
        driveSystem.continuous(velocity, turn);
    }

}
