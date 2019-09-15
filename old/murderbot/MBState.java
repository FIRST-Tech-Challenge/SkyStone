package org.eastsideprep.murderbot;

/**
 * Created by gmein on 2/6/2018.
 */

public class MBState {
    // filled in by main loop
    double right_stick_y;       // drive
    double right_stick_x;       // strafe
    double left_stick_x;        // rotate
    double right_trigger;       // fire
    double orientation;         // current gyro "heading"

    // filled in by nav control
    double heading;                 // where we are driving to right now

    // filled in by fire control thread
    volatile double orientationSweepDelta;  // fire thread can sweep robot for bursts
    volatile boolean firectrl;              // other thread is trying to ctrl robot
    volatile boolean firing;                // laser is firing right now
    double energy;                          // "charge", depleting after fire and hits
    volatile int hitCount;                  // how many times we have been hit


    MBState() {
        energy = 20;
    }
}
