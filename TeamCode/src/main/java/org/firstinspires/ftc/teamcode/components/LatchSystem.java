package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.Servo;


public class LatchSystem {
    private Servo servo;
    private final double DOWN_POSITION = 0.9;
    private final double UP_POSITION = 0.05;

    public LatchSystem(Servo servo) {
        this.servo = servo;
        initServo();
    }

    private void initServo() {
        servo.setPosition(UP_POSITION);
        servo.close();
    }

    public void run(boolean up, boolean down) {
        if (up) {
            unlatch();
        } else if (down) {
            latch();
        }
    }

    public void latch() {
        servo.setPosition(DOWN_POSITION);
        servo.close();
    }

    public void unlatch() {
        servo.setPosition(UP_POSITION);
        servo.close();
    }
}
