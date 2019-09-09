package org.darbots.darbotsftclib.libcore.tasks.chassis_tasks;

import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTeleOpControlTask;

public class GyroGuidedTurn extends RobotMotionSystemTask {
    private RobotMotionSystem m_MotionSystem;
    private RobotMotionSystemTeleOpControlTask m_TeleOpTask;
    private double m_TurnDeg;

    public GyroGuidedTurn(RobotMotionSystem MotionSystem, double turnDeg, double power){
        this.m_TeleOpTask = MotionSystem.getTeleOpTask();
        this.m_MotionSystem = MotionSystem;
        this.m_TurnDeg = turnDeg;
        double properTurnPwr = turnDeg >= 0 ? Math.abs(power) : -Math.abs(power);
        this.m_TeleOpTask.setDriveRotationSpeed(properTurnPwr);
    }
    public GyroGuidedTurn(GyroGuidedTurn Turn){
        this.m_MotionSystem = Turn.m_MotionSystem;
        this.m_TeleOpTask = this.m_MotionSystem.getTeleOpTask();
        this.m_TurnDeg = Turn.m_TurnDeg;
    }
    public double getTurnDeg(){
        return this.m_TurnDeg;
    }
    public void setTurnDeg(double turnDeg){
        this.m_TurnDeg = turnDeg;
        double properTurnPwr = turnDeg >= 0 ? Math.abs(getPower()) : -Math.abs(getPower());
    }
    public double getPower(){
        return this.m_TeleOpTask.getDriveRotationSpeed();
    }
    @Override
    protected void __startTask() {

    }

    @Override
    protected void __taskFinished() {

    }

    @Override
    public String getTaskDetailString() {
        return null;
    }

    @Override
    public void updateStatus() {

    }
}
