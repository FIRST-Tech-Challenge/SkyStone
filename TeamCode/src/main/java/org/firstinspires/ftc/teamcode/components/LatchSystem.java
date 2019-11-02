package org.firstinspires.ftc.teamcode.components;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Servo;


public class LatchSystem {
    private Servo servo;
    private final double DOWN_POSITION = 0.3;
    private final double UP_POSITION = 0.7;

    public final String TAG = "LatchSystem";

    public LatchSystem(Servo servo) {
        this.servo = servo;
        init();
    }

    public void init() {
        Log.d(TAG, "in init");
        servo.setPosition(UP_POSITION);
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
        servo.setPosition(DOWN_POSITION);
        servo.close();
    }

    public void unlatch() {
        Log.d(TAG, "unlatch");
        servo.setPosition(UP_POSITION);
        servo.close();
    }
}
