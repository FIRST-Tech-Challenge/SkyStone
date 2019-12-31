package org.firstinspires.ftc.teamcode.Experimental.AccelerometerDeadReckoning;

import android.support.annotation.NonNull;

// tracks position, velocity, and acceleration
public class MovementState {
    double x, y, z, xv, yv, zv, xa, ya, za = 0;

    public MovementState() {

    }

    public MovementState(double x, double y, double z,
                         double xv, double yv, double zv,
                         double xa, double ya, double za) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.xv = xv;
        this.yv = yv;
        this.zv = zv;
        this.xa = xa;
        this.ya = ya;
        this.za = za;
    }

    @NonNull
    @Override
    public String toString() {
        return "x: " + x + "\n" +
                "y: " + y + "\n" +
                "z: " + z + "\n" +
                "x velocity: " + xv + "\n" +
                "y velocity: " + yv + "\n" +
                "z velocity: " + zv + "\n" +
                "x acceleration: " + xa + "\n" +
                "y acceleration: " + ya + "\n" +
                "z acceleration: " + za;
    }
}