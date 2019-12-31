package org.firstinspires.ftc.teamcode.Experimental.AccelerometerDeadReckoning;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.jetbrains.annotations.NotNull;

public class PhoneAccelerometerLocalizer implements SensorEventListener {
    double previousUpdate = 0;



    MovementState motionState = new MovementState();

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (previousUpdate == 0) {
            previousUpdate = event.timestamp;
            return;
        }

        // timestamp is in nanoseconds
        double deltaTime = (event.timestamp - previousUpdate) / (1000000000);

        motionState.xa = event.values[0];
        motionState.ya = event.values[1];
        motionState.za = event.values[2];

        // basic riemann integration - maybe try trapezoid method?
        motionState.xv += motionState.xa * deltaTime;
        motionState.yv += motionState.ya * deltaTime;
        motionState.zv += motionState.za * deltaTime;

        motionState.x += motionState.xv * deltaTime;
        motionState.y += motionState.yv * deltaTime;
        motionState.z += motionState.zv * deltaTime;

        previousUpdate = event.timestamp;

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Do we care? No
    }
}
