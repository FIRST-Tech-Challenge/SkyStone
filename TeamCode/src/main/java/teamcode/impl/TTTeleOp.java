package teamcode.impl;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.common.League1TTArm;
import teamcode.common.TTDriveSystem;
import teamcode.common.TTOpMode;
import teamcode.common.Vector2;

@TeleOp(name = "TT TeleOp")
public class TTTeleOp extends TTOpMode {

    private static final double TURN_SPEED_MODIFIER = 0.6;
    private static final double REDUCED_DRIVE_SPEED = 0.4;

    private TTDriveSystem driveSystem;
    private League1TTArm arm;

    @Override
    protected void onInitialize() {
        driveSystem = new TTDriveSystem(hardwareMap);
        arm = new League1TTArm(hardwareMap);
    }

    @Override
    protected void onStart() {
        while (opModeIsActive()) {
            update();
        }
    }

    private void update() {
        driveUpdate();
        armUpdate();
    }

    protected void onStop() {
    }

    private void driveUpdate() {
        double vertical = gamepad1.right_stick_y;
        double horizontal = gamepad1.right_stick_x;
        double turn = gamepad1.left_stick_x * TURN_SPEED_MODIFIER;
        Vector2 velocity = new Vector2(horizontal, vertical);
        if (!gamepad1.right_bumper) {
            velocity = velocity.multiply(REDUCED_DRIVE_SPEED);
        }
        driveSystem.continuous(velocity, turn);
    }

    private void armUpdate() {
        while (gamepad1.y) {
            arm.lift(-1);
        }
        while (gamepad1.a) {
            arm.lift(1);
        }
        if (gamepad1.x && arm.getClawPos() != 0) {
            arm.setClawPos(0);
            sleep(100);
        } else if (gamepad1.x && arm.getClawPos() != 1) {
            arm.setClawPos(1);
            sleep(100);
        }
        arm.lift(0);
    }

}
