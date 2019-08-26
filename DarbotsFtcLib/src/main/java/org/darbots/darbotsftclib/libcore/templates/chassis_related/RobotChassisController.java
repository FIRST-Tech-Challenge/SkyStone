package org.darbots.darbotsftclib.libcore.templates.chassis_related;

import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;

public abstract class RobotChassisController implements RobotNonBlockingDevice {
    private boolean m_isGyroCalibration = true;
    private double m_LinearMotionDistanceFactor = 1.0;
    private double m_RotationalMotionDistanceFactor = 1.0;
    

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public void updateStatus() {

    }

    @Override
    public void waitUntilFinish() {

    }

    public void setGyroCalibrationEnable(boolean isEnabled){
        this.m_isGyroCalibration = isEnabled;
    }

    public boolean isGyroCalibrationEnabled(){
        return this.m_isGyroCalibration;
    }
    public void setLinearMotionDistanceFactor(double DistanceFactor){
        this.m_LinearMotionDistanceFactor = DistanceFactor;
    }
    public double getLinearMotionDistanceFacotr(){
        return this.m_LinearMotionDistanceFactor;
    }
    public void setRotationalMotionDistanceFactor(double DistanceFactor){
        this.m_RotationalMotionDistanceFactor = DistanceFactor;
    }
    public double getRotationalMotionDistanceFactor(){
        return this.m_RotationalMotionDistanceFactor;
    }


}
