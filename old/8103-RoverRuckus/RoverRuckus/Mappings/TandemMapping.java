package org.firstinspires.ftc.teamcode.RoverRuckus.Mappings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
@Config
public class TandemMapping extends ControlMapping {
    public static double INTAKE_SPEED = 0.85;
    public static double BACKWARDS_INTAKE_SPEED = 0.85;

    public static double FLIP_LEFT_FACTOR = 0.65;
    public static double FLIP_RIGHT_FACTOR = 0.4;

    public static double MAX_TURN_SPEED_F = 1;
    public static double MIN_TURN_SPEED_F = 0.2;
    public static double MIN_MOVE_SPEED = 0.4;
    public static double MAX_MOVE_SPEED = 0.8;

    public static double MAX_GP2_TURN_SPEED = 0.35;

    public static double EXPONENT = 2;
    public static double TURN_SPEED_FACTOR = 0.8;
    public static int STRAFE_FACTOR = 1;

    public static double HANG_SLOW_SPEED = 0.3;

    public int spinDir;
    private boolean g1x_down, g1b_down, g2x_down, g2b_down;
    private boolean up_down, down_down;

    public TandemMapping(Gamepad gamepad1, Gamepad gamepad2) {
        super(gamepad1, gamepad2);
    }

    private double exp(double d) {
        return Math.copySign(Math.pow(Math.abs(d), EXPONENT), d);
    }

    private int inv() {
        return gamepad1.y ? -1 : 1;
    }

    @Override
    public double driveStickX() {
        return inv() * exp(gamepad1.left_stick_x);
    }

    @Override
    public double driveStickY() {
        return exp(gamepad1.left_stick_y) * STRAFE_FACTOR;
    }

    @Override
    public double turnSpeed() {
        return removeLowVals(exp(gamepad1.right_stick_x), 0.1) * TURN_SPEED_FACTOR;
    }

    @Override
    public double translateSpeedScale() {
        if (!gamepad1.y) {
            return scaleControl(1 - gamepad1.left_trigger, MIN_MOVE_SPEED, MAX_MOVE_SPEED);
        } else {
            return HANG_SLOW_SPEED;
        }
    }

    @Override
    public double turnSpeedScale() {
        if (!gamepad1.y) {
            return scaleControl(1 - gamepad1.left_trigger, MIN_TURN_SPEED_F, MAX_TURN_SPEED_F);
        } else {
            return HANG_SLOW_SPEED;
        }
    }

    @Override
    public boolean lockTo45() {
        return gamepad1.a;
    }

    @Override
    public boolean lockTo225() {
        return gamepad1.y && !gamepad1.start;
    }

    @Override
    public boolean resetHeading() {
        return gamepad1.x && gamepad1.start;
    }

    @Override
    public double armSpeed() {
        return removeLowVals(gamepad2.left_trigger * FLIP_LEFT_FACTOR
                - gamepad2.right_trigger * FLIP_RIGHT_FACTOR, 0.05);
    }

    @Override
    public boolean collectWithArm() {
        /*if (!down_down && gamepad2.a) {
            down_down = true;
            return true;
        }
        if (!gamepad2.a && down_down) {
            down_down = false;
        }*/
        return false;
    }

    @Override
    public boolean depositWithArm() {
        /*if (!up_down && gamepad2.y) {
            up_down = true;
            // Now disable intake
            spinDir = 1;
            return true;
        }
        if (!gamepad2.y && up_down) {
            up_down = false;
        }*/
        return false;
    }

    public double getExtendSpeed() {
        return (getGP2LeftStickMode() == GP2_MODE.EXTEND) ? -clamp(gamepad2.left_stick_y) : 0;
    }

    public double getSlewSpeed() {
        return (getGP2LeftStickMode() == GP2_MODE.SLEW) ? -gamepad2.left_stick_x : 0;
    }

    enum GP2_MODE {
        EXTEND,
        SLEW
    }

    private GP2_MODE getGP2LeftStickMode() {
        return (Math.abs(gamepad2.left_stick_x) > Math.abs(gamepad2.left_stick_y)) ? GP2_MODE.SLEW : GP2_MODE.EXTEND;
    }

    public double getGP2TurnSpeed() {
        return exp(gamepad2.right_stick_x) * MAX_GP2_TURN_SPEED;
    }

    @Override
    public boolean flipOut() {
        return (gamepad2.dpad_left || gamepad1.dpad_left);
    }

    @Override
    public boolean flipBack() {
        return false;
    }

    @Override
    public boolean openLatch() {
        return gamepad1.right_trigger > 0.3;
    }

    @Override
    public boolean flipToMin() {
        return gamepad2.dpad_right;
    }

    @Override
    public boolean disableGP2Controls() {
        return gamepad2.y;
    }

    @Override
    public boolean retakeControls() { return gamepad2.start; }

    @Override
    public boolean shakeCamera() {
        return gamepad1.back;
    }

    @Override
    public double getSpinSpeed() {
        if ((gamepad2.x && !g2x_down)/* || (gamepad1.x && !g1x_down)*/) {

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

        /*if (gamepad2.b && !g2b_down) {

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
        }*/

        if (spinDir == 1) {
            return spinDir * BACKWARDS_INTAKE_SPEED;
        } else {
            return spinDir * INTAKE_SPEED;
        }
    }


    @Override
    public boolean override() {
        return gamepad1.start;
    }

    @Override
    public int getHangDir() {
        int dir;
        if (gamepad2.left_bumper || gamepad2.right_bumper) {
            dir = boolsToDir(gamepad2.left_bumper, gamepad2.right_bumper);
        } else {
            dir = boolsToDir(gamepad1.left_bumper, gamepad1.right_bumper);
        }
        if (dir != 0) {
            spinDir = 0;
        }
        return dir;
    }

    @Override
    public void setIntakeDir(int dir) {
        spinDir = dir;
    }

    @Override
    public boolean quickReverse() {
        return gamepad2.b;
    }
}
