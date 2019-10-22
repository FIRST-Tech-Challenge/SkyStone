package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.Servo;

public class LatchSystem {

    private Servo servo;
    private final double LATCH_POSITION = 0.5;
    private final double UNLATCH_POSITION = 0.0;

    public LatchSystem(Servo servo) {
        this.servo = servo;
        servo.setPosition(UNLATCH_POSITION);
    }

    public void latch() {
        servo.setPosition(LATCH_POSITION);
    }

    public void unlatch() {
        servo.setPosition(UNLATCH_POSITION);
    }
}
