package org.darbots.darbotsftclib.libcore.templates.motion_related;

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
        this.m_Speed = Speed;
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
}
