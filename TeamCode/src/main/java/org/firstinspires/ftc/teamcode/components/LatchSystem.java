package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.Servo;


public class LatchSystem {
    private Servo servoLeft;
    private Servo servoRight;
    private final double LEFT_DOWN_POSITION = 0.714623491755917;
    private final double RIGHT_DOWN_POSITION = 0.18877972287358707;
    private final double LEFT_UP_POSITION = 0.4461410963209186;
    private final double RIGHT_UP_POSITION = 0.45632352852029817;

    public LatchSystem(Servo left, Servo right) {
        this.servoLeft = left;
        this.servoRight = right;
        initServo();
    }

    private void initServo() {
        unlatch();
    }

    public void run(boolean up, boolean down) {
        if (up) {
            unlatch();
        } else if (down) {
            latch();
        }
    }

    public void latch() {
        servoLeft.setPosition(LEFT_DOWN_POSITION);
        servoLeft.close();
        servoRight.setPosition(RIGHT_DOWN_POSITION);
        servoRight.close();
    }

    public void unlatch() {
        servoLeft.setPosition(LEFT_UP_POSITION);
        servoLeft.close();
        servoRight.setPosition(RIGHT_UP_POSITION);
        servoRight.close();
    }
}