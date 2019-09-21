package org.clueless.motionplanning.field_positioning.field_position_modules.odometry.encoders;

public abstract class Encoder {

    // TODO add support for analog encoders

    int temp;

    abstract int getCurrentPosition();

    public int deltaPosition() {
        int a = getCurrentPosition() - temp;
        temp = getCurrentPosition();
        return a;
    }
}
