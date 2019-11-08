package org.darbots.darbotsftclib.libcore.templates.chassis_related;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;

public abstract class RobotMotionSystemFixedTurnTask extends RobotMotionSystemTask{
    private double m_TurnDeg;
    private double m_Speed;

    public RobotMotionSystemFixedTurnTask(double TurnDeg, double Speed){
        this.m_TurnDeg = TurnDeg;
        this.m_Speed = Speed;
    }
    public RobotMotionSystemFixedTurnTask(RobotMotionSystemFixedTurnTask Task){
        super(Task);
        this.m_TurnDeg = Task.m_TurnDeg;
        this.m_Speed = Task.m_Speed;
    }
    public double getSpeed(){
        return this.m_Speed;
    }
    public void setSpeed(double Speed){
        this.m_Speed = Math.abs(Speed);
    }
    public double getTurnDeg(){
        return this.m_TurnDeg;
    }
    public void setTurnDeg(double TurnDeg){
        this.m_TurnDeg = TurnDeg;
    }
    @Override
    public String getTaskDetailString(){
        String result = "TaskType: MotionSystemFixedTurnTask, ";
        result += "Speed: " + this.getSpeed() + ", ";
        result += "TurnDeg: " + this.getTurnDeg();
        return result;
    }
    @Override
    public void stopTask(){
        if(this.isBusy() && this.getMotionSystem().isGyroGuidedDriveEnabled()){
            this.getMotionSystem().setGyroGuidedDrivePublicStartingAngle(XYPlaneCalculations.normalizeDeg(this.getMotionSystem().getGyroGuidedDrivePublicStartingAngle() + ((float) this.getTurnDeg())));
        }
        super.stopTask();
    }
}
