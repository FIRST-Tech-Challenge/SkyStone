package org.firstinspires.ftc.teamcode.components;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LatchSystem {
    private Servo servo;
    private final double LATCH_POSITION = 0.1;
    private final double UNLATCH_POSITION = 0.9;

    public final String TAG = "LatchSystem";

    public LatchSystem(Servo servo) {
        this.servo = servo;
        init();
    }

    public void init() {
        Log.d(TAG, "in init");
        servo.setPosition(UNLATCH_POSITION);
    }

    public void run(boolean up, boolean down) {

        if (up) {
            Log.d(TAG, "in run -- up");
            latch();
        } else if (down) {
            Log.d(TAG, "in run -- down");
            unlatch();
        }

        Log.d(TAG, "servo pos: " + servo.getPosition());
    }

    public void latch() {
        Log.d(TAG,"in latch");
        servo.setPosition(LATCH_POSITION);
        servo.close();
    }

    public void unlatch() {
        Log.d(TAG, "unlatch");
        servo.setPosition(UNLATCH_POSITION);
        servo.close();
    }
}
