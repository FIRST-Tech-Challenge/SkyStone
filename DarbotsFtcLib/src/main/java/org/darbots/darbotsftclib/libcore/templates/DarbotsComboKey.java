package org.darbots.darbotsftclib.libcore.templates;

import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;

public abstract class DarbotsComboKey implements RobotNonBlockingDevice {
    private boolean m_Busy = false;

    public DarbotsComboKey(){
        this.m_Busy = false;
    }
    public DarbotsComboKey(DarbotsComboKey combo){
        this.m_Busy = false;
    }

    public void startCombo(){
        if(this.m_Busy){
            return;
        }
        this.m_Busy = true;
        this.__startCombo();
    }

    public void stopCombo(){
        if(!this.m_Busy){
            return;
        }
        this.m_Busy = false;
        this.__stopCombo();
    }

    @Override
    public boolean isBusy() {
        return this.m_Busy;
    }

    @Override
    public void waitUntilFinish() {
        while(this.isBusy()){
            if(GlobalRegister.runningOpMode != null){
                if(GlobalRegister.runningOpMode.isStarted() && (!GlobalRegister.runningOpMode.opModeIsActive())){
                    return;
                }
            }
            this.updateStatus();
        }
    }

    protected abstract void __startCombo();
    protected abstract void __stopCombo();
}
