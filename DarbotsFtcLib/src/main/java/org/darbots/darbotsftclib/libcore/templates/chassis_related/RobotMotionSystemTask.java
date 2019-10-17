/*
MIT License

Copyright (c) 2018 DarBots Collaborators

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

package org.darbots.darbotsftclib.libcore.templates.chassis_related;

import android.support.annotation.NonNull;

import org.darbots.darbotsftclib.libcore.integratedfunctions.RobotLogger;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;

public abstract class RobotMotionSystemTask implements RobotNonBlockingDevice {
    private RobotMotionSystem m_MotionSystem;
    private boolean m_IsWorking;
    public RobotMotionSystemTask(){
        this.m_IsWorking = false;
    }
    public RobotMotionSystemTask(@NonNull RobotMotionSystemTask Task){
        this.m_MotionSystem = Task.m_MotionSystem;
        this.m_IsWorking = false;
    }
    public RobotMotionSystem getMotionSystem(){
        return this.m_MotionSystem;
    }
    public void setMotionSystem(@NonNull RobotMotionSystem MotionSystem){
        this.m_MotionSystem = MotionSystem;
    }
    public void startTask(){
        if(this.m_IsWorking){
            return;
        }
        this.m_IsWorking = true;
        GlobalUtil.addLog("RobotMotionSystemTask","BeforeTask","", RobotLogger.LogLevel.DEBUG);
        GlobalUtil.addLog("RobotMotionSystemTask","TaskInfo",this.getTaskDetailString(), RobotLogger.LogLevel.DEBUG);
        this.__startTask();
    }
    protected abstract void __startTask();
    protected abstract void __taskFinished();
    public void stopTask(){
        if(!this.m_IsWorking){
            return;
        }
        GlobalUtil.addLog("RobotMotionSystemTask","AfterTask","Task ends", RobotLogger.LogLevel.DEBUG);
        this.m_IsWorking = false;
        this.__taskFinished();
        this.m_MotionSystem.__checkTasks();
    }
    @Override
    public boolean isBusy(){
        return this.m_IsWorking;
    }
    @Override
    public void waitUntilFinish(){
        while(this.isBusy()){
            this.updateStatus();
        }
    }
    public abstract String getTaskDetailString();
}
