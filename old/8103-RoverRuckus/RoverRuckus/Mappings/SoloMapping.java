package org.firstinspires.ftc.teamcode.RoverRuckus.Mappings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public class SoloMapping extends ControlMapping {
    public static double INTAKE_SPEED = 0.85;
    public static double BACKWARDS_INTAKE_SPEED = 0.85;

    public static double FLIP_LEFT_FACTOR = 0.3;
    public static double FLIP_RIGHT_FACTOR = 0.3;

    public static double MAX_TURN_SPEED = 0.5;
    public static double MAX_MOVE_SPEED = 0.3;

    public int spinDir;
    private boolean g1x_down, g1b_down, g2x_down, g2b_down;

    public SoloMapping(Gamepad gamepad1, Gamepad gamepad2) {
        super(gamepad1, gamepad2);
    }

    @Override
    public double driveStickX() {
        return gamepad1.left_stick_x;
    }

    @Override
    public double driveStickY() {
        return gamepad1.left_stick_y;
    }

    @Override
    public double turnSpeed() {
        return removeLowVals(gamepad1.right_stick_x, 0.05);
    }

    @Override
    public double translateSpeedScale() {
        return MAX_MOVE_SPEED;
    }

    @Override
    public double turnSpeedScale() {
        return MAX_TURN_SPEED;
    }

    @Override
    public double armSpeed() {
        return removeLowVals(gamepad1.left_trigger * FLIP_LEFT_FACTOR
                - gamepad1.right_trigger * FLIP_RIGHT_FACTOR, 0.05);
    }

    @Override
    public boolean lockTo45() {
        return false;
    }

    @Override
    public boolean lockTo225() {
        return false;
    }

    @Override
    public boolean resetHeading() {
        return false;
    }

    @Override
    public boolean collectWithArm() {
        return false;
    }

    @Override
    public boolean depositWithArm() {
        return false;
    }

    public double getExtendSpeed() {
        return boolsToDir(gamepad1.dpad_up, gamepad1.dpad_down);
    }

    public double getSlewSpeed() {
        return 0;
    }

    public double getGP2TurnSpeed() {
        return 0;
    }

    @Override
    public boolean flipOut() {
        return gamepad1.dpad_left;
    }

    @Override
    public boolean flipBack() {
        return false;
    }

    @Override
    public boolean openLatch() {
        return gamepad1.dpad_right;
    }

    @Override
    public boolean flipToMin() {
        return gamepad1.a;
    }

    @Override
    public boolean disableGP2Controls() {
        return true;
    }

    @Override
    public boolean retakeControls() { return false; }

    @Override
    public boolean shakeCamera() {
        return gamepad1.back;
    }

    @Override
    public double getSpinSpeed() {
        if ((gamepad2.x && !g2x_down) || (gamepad1.x && !g1x_down)) {

            spinDir = (spinDir == -1) ? 0 : -1;
            if (gamepad2.x && !g2x_down) {
                g2x_down = true;
            } else {
                g1x_down = true;
            }
        }

        if (!gamepad2.x && g2x_down) {
            g2x_down = false;
        }
        if (!gamepad1.x && g1x_down) {
            g1x_down = false;
        }

        if ((gamepad2.b && !g2b_down) || (gamepad1.b && !g1b_down)) {

            // X was just pressed
            spinDir = (spinDir == 1) ? 0 : 1;
            if (gamepad2.b && !g2b_down) {
                g2b_down = true;
            } else {
                g1b_down = true;
            }
        }

        if (!gamepad2.b && g2b_down) {
            g2b_down = false;
        }
        if (!gamepad1.b && g1b_down) {
            g1b_down = false;
        }

        if (spinDir == 1) {
            return spinDir * BACKWARDS_INTAKE_SPEED;
        } else {
            return spinDir * INTAKE_SPEED;
        }
    }


    @Override
    public boolean override() {
        return false;
    }

    @Override
    public int getHangDir() {
        return boolsToDir(gamepad1.left_bumper, gamepad1.right_bumper);
    }

    @Override
    public void setIntakeDir(int dir) {
        spinDir = dir;
    }

    @Override
    public boolean quickReverse() {return false;}
}
