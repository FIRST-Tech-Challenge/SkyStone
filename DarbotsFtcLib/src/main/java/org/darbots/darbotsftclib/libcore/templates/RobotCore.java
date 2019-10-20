package org.darbots.darbotsftclib.libcore.templates;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.integratedfunctions.RobotLogger;
import org.darbots.darbotsftclib.libcore.sensors.gyros.BNO055Gyro;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

public abstract class RobotCore implements RobotNonBlockingDevice {
    private RobotLogger m_Logger;
    private RobotGyro m_Gyro;
    public RobotCore(String logFileName, HardwareMap hardwareMap){
        m_Logger = new RobotLogger(logFileName);
        m_Gyro = new BNO055Gyro(hardwareMap,"imu");
    }
    public abstract void stop();
    public abstract void terminate();
    public RobotLogger getLogger(){
        return this.m_Logger;
    }
    public abstract RobotMotionSystem getChassis();
    @Override
    public void waitUntilFinish(){
        while(this.isBusy()){
            this.updateStatus();
        }
    }
    public RobotGyro getGyro(){
        return this.m_Gyro;
    }
}
