package org.darbots.darbotsftclib.libcore.templates.other_sensors;

import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;

public abstract class RobotGyro implements RobotNonBlockingDevice {
    public enum HeadingRotationPositiveOrientation{
        CounterClockwise,
        Clockwise
    }

    protected abstract void updateData();
    @Override
    public void updateStatus(){
        this.updateData();
    }
    @Override
    public boolean isBusy(){
        return false;
    }
    @Override
    public void waitUntilFinish(){
        return;
    }
    public abstract float getHeading();
    public abstract HeadingRotationPositiveOrientation getHeadingRotationPositiveOrientation();

}
