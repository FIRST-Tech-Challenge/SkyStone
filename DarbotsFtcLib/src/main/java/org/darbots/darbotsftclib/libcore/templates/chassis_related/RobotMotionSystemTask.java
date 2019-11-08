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

import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.RobotLogger;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

public abstract class RobotMotionSystemTask implements RobotNonBlockingDevice {
    private RobotMotionSystem m_MotionSystem;
    private boolean m_IsWorking;
    private boolean m_CustomSteadilySpeedUpEnabled = false;
    private double m_CustomSteadilySpeedUpStartingSpeed = 0.25;
    private double m_CustomSteadilySpeedUpEndingSpeed = 0.25;
    private double m_CustomSteadilySpeedUpStartingZoneRatio = 0.3;
    private double m_CustomSteadilySpeedUpEndingZoneRatio = 0.3;
    private float m_GyroStartAng = 0.0f;

    public RobotMotionSystemTask(){
        this.m_IsWorking = false;
    }
    public RobotMotionSystemTask(@NonNull RobotMotionSystemTask Task){
        this.m_MotionSystem = Task.m_MotionSystem;
        this.m_IsWorking = false;
        this.m_CustomSteadilySpeedUpEnabled = Task.m_CustomSteadilySpeedUpEnabled;
        this.m_CustomSteadilySpeedUpStartingSpeed = Task.m_CustomSteadilySpeedUpStartingSpeed;
        this.m_CustomSteadilySpeedUpEndingSpeed = Task.m_CustomSteadilySpeedUpEndingSpeed;
        this.m_CustomSteadilySpeedUpStartingZoneRatio = Task.m_CustomSteadilySpeedUpStartingZoneRatio;
        this.m_CustomSteadilySpeedUpEndingZoneRatio = Task.m_CustomSteadilySpeedUpEndingZoneRatio;
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
        if((!this.getMotionSystem().isGyroGuidedDriveEnabled()) && GlobalUtil.getGyro() != null){
            RobotGyro globalGyro = GlobalUtil.getGyro();
            globalGyro.updateStatus();
            this.m_GyroStartAng = globalGyro.getHeading();
        } else if(this.getMotionSystem().isGyroGuidedDriveEnabled()){
            this.m_GyroStartAng = this.getMotionSystem().getGyroGuidedDrivePublicStartingAngle();
        }
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
            if(GlobalRegister.runningOpMode != null){
                if(GlobalRegister.runningOpMode.isStarted() && (!GlobalRegister.runningOpMode.opModeIsActive())){
                    return;
                }
            }
            this.updateStatus();
        }
    }
    public abstract String getTaskDetailString();
    public abstract double getTaskProgressRatio();
    public boolean isCustomSteadilySpeedUpEnabled(){
        return this.m_CustomSteadilySpeedUpEnabled;
    }
    public void setCustomSteadilySpeedUpEnabled(boolean enabled){
        this.m_CustomSteadilySpeedUpEnabled = enabled;
    }
    public double getCustomSteadilySpeedUpStartingSpeed(){
        return this.m_CustomSteadilySpeedUpStartingSpeed;
    }
    public void setCustomSteadilySpeedUpStartingSpeed(double speed){
        this.m_CustomSteadilySpeedUpStartingSpeed = Math.abs(speed);
    }
    public double getCustomSteadilySpeedUpEndingSpeed(){
        return this.m_CustomSteadilySpeedUpEndingSpeed;
    }
    public void setCustomSteadilySpeedUpEndingSpeed(double speed){
        this.m_CustomSteadilySpeedUpEndingSpeed = Math.abs(speed);
    }
    public void setCustomSteadilySpeedUpBothSpeed(double speed){
        this.setCustomSteadilySpeedUpStartingSpeed(speed);
        this.setCustomSteadilySpeedUpEndingSpeed(speed);
    }
    public double getCustomSteadilySpeedUpStartingZoneRatio(){
        return this.m_CustomSteadilySpeedUpStartingZoneRatio;
    }
    public void setCustomSteadilySpeedUpStartingZoneRatio(double Ratio){
        this.m_CustomSteadilySpeedUpStartingZoneRatio = Math.abs(Ratio);
    }
    public double getCustomSteadilySpeedUpEndingZoneRatio(){
        return this.m_CustomSteadilySpeedUpEndingZoneRatio;
    }
    public void setCustomSteadilySpeedUpEndingZoneRatio(double Ratio){
        this.m_CustomSteadilySpeedUpEndingZoneRatio = Math.abs(Ratio);
    }
    public void setCustomSteadilySpeedUpBothRatio(double Ratio){
        this.setCustomSteadilySpeedUpStartingZoneRatio(Ratio);
        this.setCustomSteadilySpeedUpEndingZoneRatio(Ratio);
    }
    protected double __getSupposedSteadilySpeedUpAbsSpeed(double AbsSpeed){
        if(!this.isBusy()){
            return AbsSpeed;
        }

        double steadySpeedUpStartingSpeed = 0;
        double steadySpeedUpEndingSpeed = 0;
        double steadySpeedUpStartingZone = 0;
        double steadySpeedUpEndingZone = 0;
        if(this.isCustomSteadilySpeedUpEnabled()) {
            steadySpeedUpStartingSpeed = this.getCustomSteadilySpeedUpStartingSpeed();
            steadySpeedUpEndingSpeed = this.getCustomSteadilySpeedUpEndingSpeed();
            steadySpeedUpStartingZone = this.getCustomSteadilySpeedUpStartingZoneRatio();
            steadySpeedUpEndingZone = this.getCustomSteadilySpeedUpEndingZoneRatio();
        }else{
            steadySpeedUpStartingSpeed = this.getMotionSystem().getSteadySpeedUpThreshold();
            steadySpeedUpEndingSpeed = this.getMotionSystem().getSteadySpeedUpThreshold();
            steadySpeedUpStartingZone = this.getMotionSystem().getSteadySpeedUpZoneRatio();
            steadySpeedUpEndingZone = this.getMotionSystem().getSteadySpeedUpZoneRatio();
        }
        if(this.isCustomSteadilySpeedUpEnabled() || (this.getMotionSystem().isSteadySpeedUp() && AbsSpeed > this.getMotionSystem().getSteadySpeedUpThreshold())){
            double ExtraSpeed = 0;

            double jobPercentile = this.getTaskProgressRatio();

            if(jobPercentile < steadySpeedUpStartingZone){
                ExtraSpeed = AbsSpeed - steadySpeedUpStartingSpeed;
                AbsSpeed = steadySpeedUpStartingSpeed + (jobPercentile / steadySpeedUpStartingZone) * ExtraSpeed;
            }else if(jobPercentile > steadySpeedUpEndingZone){
                ExtraSpeed = AbsSpeed - steadySpeedUpEndingSpeed;
                AbsSpeed = steadySpeedUpEndingSpeed + ((1.0 - jobPercentile) / steadySpeedUpEndingZone) * ExtraSpeed;
            }
        }
        return AbsSpeed;
    }
    protected double __getGyroGuidedDeltaSpeed(double AbsSpeed){
        if(!this.isBusy()){
            return 0;
        }
        if(GlobalUtil.getGyro() == null || (!this.getMotionSystem().isGyroGuidedDriveEnabled())){
            return 0;
        }
        RobotGyro globalGyro = GlobalUtil.getGyro();
        globalGyro.updateStatus();
        double currentAng = globalGyro.getHeading();
        double deltaAng = XYPlaneCalculations.normalizeDeg(currentAng - m_GyroStartAng);
        if (globalGyro.getHeadingRotationPositiveOrientation() == RobotGyro.HeadingRotationPositiveOrientation.Clockwise) {
            deltaAng = -deltaAng;
        }
        double absDeltaAng = Math.abs(deltaAng);
        double deltaSpeedEachSide = 0;
        if (absDeltaAng >= 5) {
            deltaSpeedEachSide = Range.clip(0.4 * AbsSpeed, 0, 0.2);
        } else if (absDeltaAng >= 1.5) {
            deltaSpeedEachSide = Range.clip(0.2 * AbsSpeed, 0, 0.15);
        } else if (absDeltaAng >= 0.5) {
            deltaSpeedEachSide = Range.clip(0.1 * AbsSpeed, 0, 0.1);
        }
        if (deltaSpeedEachSide < 0.02 && deltaSpeedEachSide != 0) {
            deltaSpeedEachSide = 0.02;
        }
        if (deltaAng > 0) {
            return -deltaSpeedEachSide;
        } else if (deltaAng < 0) {
            return deltaSpeedEachSide;
        }
        return 0;
    }
}
