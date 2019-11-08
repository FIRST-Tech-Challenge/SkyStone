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

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.odometry.Robot2DPositionTracker;

import java.util.ArrayList;

public abstract class RobotMotionSystem implements RobotNonBlockingDevice {
    private ArrayList<RobotMotionSystemTask> m_TaskLists;
    private Robot2DPositionTracker m_PosTracker;
    private double m_LinearZMotionDistanceFactor;
    private double m_LinearXMotionDistanceFactor;
    private double m_RotationalMotionDistanceFactor;
    private boolean m_FixedDistanceGyroGuidedDrive = false;
    private float m_GyroGuidedDrivePublicStartingAngle = -360;
    private boolean m_SteadySpeedUp = true;
    private double m_SteadySpeedUpThreshold = 0.25;
    private double m_SteadySpeedUpZoneRatio = 0.35;
    public RobotMotionSystem(Robot2DPositionTracker PositionTracker){
        this.m_TaskLists = new ArrayList();
        this.m_PosTracker = PositionTracker;
        this.setLinearMotionDistanceFactor(1);
        this.m_RotationalMotionDistanceFactor = 1;
    }
    public RobotMotionSystem(RobotMotionSystem MotionSystem){
        this.m_TaskLists = new ArrayList();
        this.m_PosTracker = MotionSystem.m_PosTracker;
        this.m_LinearZMotionDistanceFactor = MotionSystem.m_LinearZMotionDistanceFactor;
        this.m_LinearXMotionDistanceFactor = MotionSystem.m_LinearXMotionDistanceFactor;
        this.m_RotationalMotionDistanceFactor = MotionSystem.m_RotationalMotionDistanceFactor;
        this.m_FixedDistanceGyroGuidedDrive = MotionSystem.m_FixedDistanceGyroGuidedDrive;
        this.m_GyroGuidedDrivePublicStartingAngle = MotionSystem.m_GyroGuidedDrivePublicStartingAngle;
        this.m_SteadySpeedUp = MotionSystem.m_SteadySpeedUp;
        this.m_SteadySpeedUpThreshold = MotionSystem.m_SteadySpeedUpThreshold;
        this.m_SteadySpeedUpZoneRatio = MotionSystem.m_SteadySpeedUpZoneRatio;
    }

    public double getLinearZMotionDistanceFactor(){
        return this.m_LinearZMotionDistanceFactor;
    }
    public void setLinearZMotionDistanceFactor(double Factor){
        this.m_LinearZMotionDistanceFactor = Factor;
    }

    public double getLinearXMotionDistanceFactor(){
        return this.m_LinearXMotionDistanceFactor;
    }
    public void setLinearXMotionDistanceFactor(double Factor){
        this.m_LinearXMotionDistanceFactor = Factor;
    }
    public void setLinearMotionDistanceFactor(double Factor){
        this.setLinearZMotionDistanceFactor(Factor);
        this.setLinearXMotionDistanceFactor(Factor);
    }

    public double getRotationalMotionDistanceFactor(){
        return this.m_RotationalMotionDistanceFactor;
    }

    public void setRotationalMotionDistanceFactor(double Factor){
        this.m_RotationalMotionDistanceFactor = Factor;
    }

    public Robot2DPositionTracker getPositionTracker(){
        return this.m_PosTracker;
    }
    public void setPositionTracker(Robot2DPositionTracker PositionTracker){
        this.m_PosTracker = PositionTracker;
    }
    public void addTask(@NonNull RobotMotionSystemTask Task){
        this.m_TaskLists.add(Task);
        this.scheduleTasks();
    }

    public void replaceTask(@NonNull RobotMotionSystemTask Task){
        if(!this.m_TaskLists.isEmpty()){
             if(this.m_TaskLists.get(0).isBusy())
                 this.m_TaskLists.get(0).stopTask();
        }
        this.m_TaskLists.clear();
        this.m_TaskLists.add(Task);
        this.scheduleTasks();
    }

    public void deleteCurrentTask(){
        if(this.m_TaskLists.isEmpty()){
            this.__stopMotion();
            return;
        }
        this.m_TaskLists.get(0).stopTask();
        this.m_TaskLists.remove(0);
        this.scheduleTasks();
    }

    public ArrayList<RobotMotionSystemTask> getTaskLists(){
        return this.m_TaskLists;
    }

    public RobotMotionSystemTask getCurrentTask(){
        return this.m_TaskLists.isEmpty() ? null : this.m_TaskLists.get(0);
    }

    public void deleteAllTasks(){
        if(this.m_TaskLists.isEmpty()){
            this.__stopMotion();
            return;
        }
        RobotMotionSystemTask currentTask = this.m_TaskLists.get(0);
        this.m_TaskLists.get(0).stopTask();
        this.m_TaskLists.clear();
        this.__stopMotion();
    }

    public void __checkTasks(){
        if(this.m_TaskLists.isEmpty()){
            this.__stopMotion();
            return;
        }else if(!this.m_TaskLists.get(0).isBusy()){
            this.deleteCurrentTask();
        }
    }

    protected abstract void __stopMotion();

    protected void scheduleTasks(){
        if(!this.m_TaskLists.isEmpty()){
            if(!this.m_TaskLists.get(0).isBusy()) {
                this.m_TaskLists.get(0).setMotionSystem(this);
                this.m_TaskLists.get(0).startTask();
            }
        }else{
            this.__stopMotion();
        }
    }

    @Override
    public boolean isBusy(){
        return (!this.m_TaskLists.isEmpty());
    }

    @Override
    public void updateStatus(){
        if((!this.m_TaskLists.isEmpty())){
            if(this.m_TaskLists.get(0).isBusy())
                this.m_TaskLists.get(0).updateStatus();
        }
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


    public boolean isGyroGuidedDriveEnabled(){
        return this.m_FixedDistanceGyroGuidedDrive;
    }

    public void setGyroGuidedDriveEnabled(boolean Enabled){
        this.m_FixedDistanceGyroGuidedDrive = Enabled;
        if(Enabled && GlobalUtil.getGyro() != null && this.m_GyroGuidedDrivePublicStartingAngle == -360){
            updateGyroGuidedPublicStartingAngle();
        }
    }

    public void updateGyroGuidedPublicStartingAngle(){
        if(GlobalUtil.getGyro() != null){
            GlobalUtil.getGyro().updateStatus();
            this.m_GyroGuidedDrivePublicStartingAngle = GlobalUtil.getGyro().getHeading();
        }
    }

    public float getGyroGuidedDrivePublicStartingAngle(){
        if(this.m_GyroGuidedDrivePublicStartingAngle == -360){
            return 0;
        }
        return this.m_GyroGuidedDrivePublicStartingAngle;
    }

    public void setGyroGuidedDrivePublicStartingAngle(float Ang){
        this.m_GyroGuidedDrivePublicStartingAngle = XYPlaneCalculations.normalizeDeg(Ang);
    }

    public boolean isSteadySpeedUp(){
        return this.m_SteadySpeedUp;
    }

    public void setSteadySpeedUp(boolean Enabled){
        this.m_SteadySpeedUp = Enabled;
    }

    public double getSteadySpeedUpThreshold(){
        return this.m_SteadySpeedUpThreshold;
    }

    public void setSteadySpeedUpThreshold(double Threshold){
        this.m_SteadySpeedUpThreshold = Math.abs(Threshold);
    }

    public double getSteadySpeedUpZoneRatio(){
        return this.m_SteadySpeedUpZoneRatio;
    }

    public void setSteadySpeedUpZoneRatio(double Ratio){
        this.m_SteadySpeedUpZoneRatio = Math.abs(Ratio);
    }

    public abstract RobotMotionSystemFixedXDistanceTask getFixedXDistanceTask(double XDistance, double Speed);
    public abstract RobotMotionSystemFixedZDistanceTask getFixedZDistanceTask(double ZDistance, double Speed);
    public abstract RobotMotionSystemFixedTurnTask getFixedTurnTask(double Deg, double Speed);
    public abstract RobotMotionSystemTeleOpControlTask getTeleOpTask();
}
