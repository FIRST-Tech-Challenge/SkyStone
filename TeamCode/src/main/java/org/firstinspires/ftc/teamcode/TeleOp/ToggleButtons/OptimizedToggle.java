package org.firstinspires.ftc.teamcode.TeleOp.ToggleButtons;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.nio.ByteBuffer;

public class OptimizedToggle {

    private static final int BUTTONS_OFFSET = 37; // Change if serialization in Gamepad class changes

    private enum Situation {
        ONE_BUTTON_MOTOR, ONE_BUTTON_SERVO, TWO_BUTTON_MOTOR, TWO_BUTTON_SERVO
    }

    private DcMotor[] motors;
    private Servo[] servos;
    private double[][] powOrPos1;
    private double[] powOrPos2;
    private boolean blocker;
    private OptimizedGamepadButtons button1;
    private OptimizedGamepadButtons button2;
    private Gamepad gamepadSituation1;
    private Gamepad gamepadSituation2;

    private int gamepadState1;
    private int gamepadState2;
    private Situation situation;

    private boolean mvmt1 = false;
    private boolean mvmt2 = false;

    public OptimizedToggle(Gamepad gamepad, OptimizedGamepadButtons toggleButton, DcMotor[] motors,
                       double[][] power) {
        this.motors = motors;
        this.gamepadSituation1 = gamepad;
        powOrPos1 = power;
        button1 = toggleButton;
        blocker = false;
        situation = Situation.ONE_BUTTON_MOTOR;
    }

    public OptimizedToggle(Gamepad gamepad, OptimizedGamepadButtons toggleButton, Servo[] servos,
                       double[][] positions) {
        this.servos = servos;
        this.gamepadSituation1 = gamepad;
        powOrPos1 = positions;
        button1 = toggleButton;
        blocker = false;
        situation = Situation.ONE_BUTTON_SERVO;
    }

    public OptimizedToggle(Gamepad gamepadNo1, Gamepad gamepadNo2, OptimizedGamepadButtons toggleButton1,
                           OptimizedGamepadButtons toggleButton2, DcMotor[] motors, double[][] power1, double[] power2) {
        this.motors = motors;
        gamepadSituation1 = gamepadNo1;
        gamepadSituation2 = gamepadNo2;
        powOrPos1 = power1;
        powOrPos2 = power2;
        button1 = toggleButton1;
        button2 = toggleButton2;
        blocker = false;
        situation = Situation.TWO_BUTTON_MOTOR;
    }

    public OptimizedToggle(Gamepad gamepadNo1, Gamepad gamepadNo2, OptimizedGamepadButtons toggleButton1,
                           OptimizedGamepadButtons toggleButton2, Servo[] servos, double[][] position1, double[] position2) {
        this.servos = servos;
        gamepadSituation1 = gamepadNo1;
        gamepadSituation2 = gamepadNo2;
        powOrPos1 = position1;
        powOrPos2 = position2;
        button1 = toggleButton1;
        button2 = toggleButton2;
        blocker = false;
        situation = Situation.TWO_BUTTON_SERVO;
    }

    public void getGamepadStateAndRun() {
        try {
            // bits 31-32 left, right bumper
            // bits 28-30 guide, start, back
            // bits 23-27 a, b, x, y
            // bits 19-22 dpad_up, down, left, right
            // bits 17-18 left stick button, right stick button
            // bits 15-16 left trigger, right trigger
            gamepadState1 = ByteBuffer.wrap(gamepadSituation1.toByteArray(), BUTTONS_OFFSET, BUTTONS_OFFSET + 4).getInt();
            gamepadState2 = ByteBuffer.wrap(gamepadSituation2.toByteArray(), BUTTONS_OFFSET, BUTTONS_OFFSET + 4).getInt();

            // Add trigger data

            gamepadState1 += (gamepadSituation1.right_trigger >= 0.5 ? 1 : 0) << 16;
            gamepadState1 += (gamepadSituation1.left_trigger >= 0.5 ? 1 : 0) << 17;

            gamepadState2 += (gamepadSituation2.right_trigger >= 0.5 ? 1 : 0) << 16;
            gamepadState2 += (gamepadSituation2.left_trigger >= 0.5 ? 1 : 0) << 17;
        } catch (RobotCoreException e) {
            RobotLog.e("Error while ");
        }
        if (situation == Situation.ONE_BUTTON_MOTOR || situation == Situation.ONE_BUTTON_SERVO)
            oneButtonHandler();
        else if (situation == Situation.TWO_BUTTON_MOTOR || situation == Situation.TWO_BUTTON_SERVO)
            twoButtonHandler();
    }

    public boolean[] isRunning() {
        if (situation == Situation.ONE_BUTTON_MOTOR || situation == Situation.ONE_BUTTON_SERVO)
            return new boolean[]{mvmt1};
        else
            return new boolean[]{mvmt1, mvmt2};
    }

    public OptimizedGamepadButtons[] getButtons() {
        if (situation == Situation.ONE_BUTTON_MOTOR || situation == Situation.ONE_BUTTON_SERVO)
            return new OptimizedGamepadButtons[]{button1};
        else
            return new OptimizedGamepadButtons[]{button1, button2};
    }

    public boolean controllingServos() {
        if (situation == Situation.ONE_BUTTON_SERVO || situation == Situation.TWO_BUTTON_SERVO)
            return true;
        else
            return false;
    }

    public boolean controllingDcMotors() {
        if (situation == Situation.ONE_BUTTON_SERVO || situation == Situation.TWO_BUTTON_SERVO)
            return false;
        else
            return true;
    }

    public Servo[] getServos() {
        return servos;
    }

    public DcMotor[] getMotors() {
        return motors;
    }

    private void oneButtonLogic() {
        if (!blocker) {
            if (mvmt1)
                mvmt1 = false;
            else
                mvmt1 = true;
            blocker = true;
        }
    }

    private void twoButtonLogic1() {
        if (!blocker) {
            if (!mvmt1) {
                mvmt1 = true;
                mvmt2 = false;
            } else {
                mvmt1 = false;
                mvmt2 = false;
            }
            blocker = true;
        }
    }

    private void twoButtonLogic2() {
        if (!blocker) {
            if (!mvmt2) {
                mvmt1 = false;
                mvmt2 = true;
            } else {
                mvmt1 = false;
                mvmt2 = false;
            }
            blocker = true;
        }
    }

    private void oneButtonServoLogicAdditional() {
        if (mvmt1) {
            for (int i = 0; i < servos.length; i++) {
                servos[i].setPosition(powOrPos1[i][0]);
            }
        }

        if (!mvmt1) {
            for (int i = 0; i < servos.length; i++) {
                servos[i].setPosition(powOrPos1[i][1]);
            }
        }
    }

    private void oneButtonMotorLogicAdditional() {
        if (mvmt1) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(powOrPos1[i][0]);
            }
        }

        if (!mvmt1) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(powOrPos1[i][1]);
            }
        }
    }

    private void twoButtonServoLogicAdditional() {
        if (mvmt1) {
            for (int i = 0; i < servos.length; i++) {
                servos[i].setPosition(powOrPos1[i][0]);
            }
        }

        if (mvmt2) {
            for (int i = 0; i < servos.length; i++) {
                servos[i].setPosition(powOrPos2[i]);
            }
        }

        if (!mvmt1 && !mvmt2) {
            for (int i = 0; i < servos.length; i++) {
                servos[i].setPosition(powOrPos1[i][1]);
            }
        }
    }

    private void twoButtonMotorLogicAdditional() {
        if (mvmt1) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(powOrPos1[i][0]);
            }
        }

        if (mvmt2) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(powOrPos2[i]);
            }
        }

        if (!mvmt1 && !mvmt2) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(powOrPos1[i][1]);
            }
        }
    }

    private void oneButtonHandler() {
        if ((button1.bitmask | gamepadState1) != 0) {
            oneButtonLogic();
        } else {
            blocker = false;
        }

        if (situation == Situation.ONE_BUTTON_MOTOR)
            oneButtonMotorLogicAdditional();
        else if (situation == Situation.ONE_BUTTON_SERVO)
            oneButtonServoLogicAdditional();
    }

    private void twoButtonHandler() {

        boolean b1, b2;

        if (b1 = (button1.bitmask | gamepadState1) != 0)
            twoButtonLogic1();
        if (b2 = (button2.bitmask | gamepadState2) != 0)
            twoButtonLogic2();
        if (!b1 && !b2) {
            blocker = false;
        }
        if (situation == Situation.TWO_BUTTON_MOTOR)
            twoButtonMotorLogicAdditional();
        else if (situation == Situation.TWO_BUTTON_SERVO)
            twoButtonServoLogicAdditional();
    }
}