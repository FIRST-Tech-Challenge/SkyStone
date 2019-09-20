package teamcode.impl;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.common.TTArm;
import teamcode.common.TTHardwareManager;
import teamcode.common.TTOpMode;

/*
 Linear lift lift is controlled by triggers.
 Arm rotation is DPad.
 B Button moves 15 degrees and Y Button moves 45 degrees.
 */
@TeleOp(name = "TT Arm Test")
public class ArmTest extends TTOpMode {

    private TTArm arm;

    @Override
    protected void onInitialize() {
        setHardwareRestriction(TTHardwareManager.TTHardwareRestriction.ARM_ONLY);
    }

    @Override
    protected void onStart() {
        arm = getRobot().getArm();
        while (opModeIsActive()) {
            update();
        }
    }

    private void update() {
        while (gamepad1.right_trigger > 0.3) {
            arm.liftContinuous(gamepad1.right_trigger);
        }
        arm.liftContinuous(0);

        while (gamepad1.left_trigger > 0.3) {
            arm.liftContinuous(-gamepad1.left_trigger);
        }
        arm.liftContinuous(0);
        while (gamepad1.right_bumper) {
            arm.intake(0.75);
        }
        arm.intake(0);
        while (gamepad1.left_bumper) {
            arm.intake(-0.75);
        }
        arm.intake(0);

        if (gamepad1.dpad_down) {
            while (gamepad1.dpad_down) {
                arm.rotateContinuous(-0.75);
            }
            arm.rotateContinuous(0);
        } else if (gamepad1.dpad_up) {

            while (gamepad1.dpad_up) {
                arm.rotateContinuous(0.75);
            }
            arm.rotateContinuous(0);
        } else if (gamepad1.b) {
            arm.setLiftHeight(arm.getLiftHeight() + 3, 1);
            // arm.rotate(15,1);
        } else if (gamepad1.a) {
            arm.rotate(-45, 1);
        } else if (gamepad1.x) {
            if (arm.getClawPosition() == 1.0) {
                arm.setClawPosition(0);
            } else {
                arm.setClawPosition(1);
            }
        }
    }


    public void DPadArmMovement(boolean up) {
        boolean down = !up;

        if (down && gamepad1.dpad_down) {
            while (gamepad1.dpad_down) {
                arm.rotateContinuous(-1);
            }
            arm.rotateContinuous(0);

        } else if (up && gamepad1.dpad_up) {
            while (gamepad1.dpad_up) {
                arm.rotateContinuous(1);
            }
            arm.rotateContinuous(0);
        }
    }
}
