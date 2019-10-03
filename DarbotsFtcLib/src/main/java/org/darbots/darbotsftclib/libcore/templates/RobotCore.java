package org.darbots.darbotsftclib.libcore.templates;

import org.darbots.darbotsftclib.libcore.integratedfunctions.RobotLogger;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;

public abstract class RobotCore implements RobotNonBlockingDevice {
    private RobotLogger m_Logger;
    public RobotCore(String logFileName){
        m_Logger = new RobotLogger(logFileName);
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
}
