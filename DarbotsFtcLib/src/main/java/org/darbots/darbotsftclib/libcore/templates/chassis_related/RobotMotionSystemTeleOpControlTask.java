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

public abstract class RobotMotionSystemTeleOpControlTask extends RobotMotionSystemTask {
    private double m_DriveXSpeed;
    private double m_DriveZSpeed;
    private double m_DriveRotationSpeed;

    public RobotMotionSystemTeleOpControlTask(){
        this.m_DriveXSpeed = 0;
        this.m_DriveZSpeed = 0;
        this.m_DriveRotationSpeed = 0;
    }

    public RobotMotionSystemTeleOpControlTask(RobotMotionSystemTeleOpControlTask Task){
        super(Task);
        this.m_DriveXSpeed = Task.m_DriveXSpeed;
        this.m_DriveZSpeed = Task.m_DriveZSpeed;
        this.m_DriveRotationSpeed = Task.m_DriveRotationSpeed;
    }

    public double getDriveXSpeed(){
        return this.m_DriveXSpeed;
    }

    public void setDriveXSpeed(double XSpeed){
        this.m_DriveXSpeed = XSpeed;
    }

    public double getDriveZSpeed(){
        return this.m_DriveZSpeed;
    }

    public void setDriveZSpeed(double ZSpeed){
        this.m_DriveZSpeed = ZSpeed;
    }

    public double getDriveRotationSpeed(){
        return this.m_DriveRotationSpeed;
    }

    public void setDriveRotationSpeed(double RotationSpeed){
        this.m_DriveRotationSpeed = RotationSpeed;
    }

    protected abstract void __updateDriveSpeedAndPositionTracker();
    protected abstract void __startDrive();

    @Override
    protected void __startTask() {
        this.__startDrive();
        this.__updateDriveSpeedAndPositionTracker();
    }

    @Override
    public void updateStatus() {
        this.__updateMotorStatus();
        if(this.isBusy()) {
            this.__updateDriveSpeedAndPositionTracker();
        }
    }
    protected abstract void __updateMotorStatus();
    @Override
    public String getTaskDetailString(){
        String result = "TaskType: MotionSystemTeleOpControlTask, ";
        result += "DriveXSpeed: " + this.getDriveXSpeed() + ", ";
        result += "DriveZSpeed: " + this.getDriveZSpeed() + ", ";
        result += "TurnSpeed: " + this.getDriveRotationSpeed();
        return result;
    }
}
