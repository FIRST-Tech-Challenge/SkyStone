package org.darbots.darbotsftclib.libcore.templates.motion_related;

public abstract class RobotMotionSystemFixedXDistanceTask extends RobotMotionSystemTask {
    private double m_XDistance;
    private double m_Speed;

    public RobotMotionSystemFixedXDistanceTask(double XDistance, double Speed){
        this.m_XDistance = XDistance;
        this.m_Speed = Speed;
    }
    public RobotMotionSystemFixedXDistanceTask(RobotMotionSystemFixedXDistanceTask Task){
        super(Task);
        this.m_XDistance = Task.m_XDistance;
        this.m_Speed = Task.m_Speed;
    }
    public double getSpeed(){
        return this.m_Speed;
    }
    public void setSpeed(double Speed){
        this.m_Speed = Speed;
    }
    public double getXDistance(){
        return this.m_XDistance;
    }
    public void setXDistance(double Distance){
        this.m_XDistance = Distance;
    }
    @Override
    public String getTaskDetailString(){
        String result = "TaskType: MotionSystemFixedXDistanceTask, ";
        result += "Speed: " + this.getSpeed() + ", ";
        result += "XDistance: " + this.getXDistance();
        return result;
    }
}
