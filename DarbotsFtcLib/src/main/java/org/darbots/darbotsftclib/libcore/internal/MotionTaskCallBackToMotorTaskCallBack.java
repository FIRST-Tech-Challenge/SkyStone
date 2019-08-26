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

package org.darbots.darbotsftclib.libcore.internal;

import android.support.annotation.NonNull;

import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.templates.motion_related.RobotMotionTaskCallBack;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotorTaskCallBack;

public class MotionTaskCallBackToMotorTaskCallBack implements RobotMotorTaskCallBack {
    protected RobotMotion m_Motion;
    protected RobotMotionTaskCallBack m_TaskCallBack;
    public MotionTaskCallBackToMotorTaskCallBack(@NonNull RobotMotion motionController, RobotMotionTaskCallBack MotionTaskCB){
        this.m_Motion = motionController;
        this.m_TaskCallBack = MotionTaskCB;
    }
    public RobotMotion getMotionController(){
        return this.m_Motion;
    }
    public void setMotionController(@NonNull RobotMotion motionController){
        this.m_Motion = motionController;
    }
    public RobotMotionTaskCallBack getTaskCallBack(){
        return this.m_TaskCallBack;
    }
    public void setTaskCallBack(RobotMotionTaskCallBack TaskCallBack){
        this.m_TaskCallBack = TaskCallBack;
    }
    @Override
    public void finishRunning(RobotMotorController Controller, boolean timeOut, double timeUsedInSec, int CountsMoved) {
        if(this.m_TaskCallBack != null){
            this.m_TaskCallBack.finishRunning(this.m_Motion,timeOut,timeUsedInSec,CountsMoved,CountsMoved / Controller.getMotor().getMotorType().getCountsPerRev() * this.m_Motion.getRobotWheel().getCircumference());
        }
    }
}
