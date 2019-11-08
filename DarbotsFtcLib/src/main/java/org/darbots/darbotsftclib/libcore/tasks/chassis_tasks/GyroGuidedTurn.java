package org.darbots.darbotsftclib.libcore.tasks.chassis_tasks;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTeleOpControlTask;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

public class GyroGuidedTurn extends RobotMotionSystemTask {
    private RobotMotionSystem m_MotionSystem;
    private RobotMotionSystemTeleOpControlTask m_TeleOpTask;
    private double m_TurnDeg;
    private double m_StartDeg;
    private RobotGyro m_Gyro;
    private double m_ErrorMarginAng = 0.2;

    public GyroGuidedTurn(RobotMotionSystem MotionSystem, RobotGyro Gyro, double turnDeg, double power){
        this.m_TeleOpTask = MotionSystem.getTeleOpTask();
        this.m_MotionSystem = MotionSystem;
        this.m_TurnDeg = turnDeg;
        this.m_Gyro = Gyro;
        this.setPower(power);
    }
    public GyroGuidedTurn(GyroGuidedTurn Turn){
        this.m_MotionSystem = Turn.m_MotionSystem;
        this.m_TeleOpTask = this.m_MotionSystem.getTeleOpTask();
        this.m_TeleOpTask.setDriveRotationSpeed(Turn.m_TeleOpTask.getDriveRotationSpeed());
        this.m_TurnDeg = Turn.m_TurnDeg;
        this.m_Gyro = Turn.m_Gyro;
        this.m_ErrorMarginAng = Turn.m_ErrorMarginAng;
    }
    public double getTurnDeg(){
        return this.m_TurnDeg;
    }
    public void setTurnDeg(double turnDeg){
        this.m_TurnDeg = XYPlaneCalculations.normalizeDeg(turnDeg);
    }
    public double getErrorMarginAngle(){
        return this.m_ErrorMarginAng;
    }
    public void setErrorMarginAngle(double Margin){
        this.m_ErrorMarginAng = Math.abs(Margin);
    }

    public double getPower(){
        return this.m_TeleOpTask.getDriveRotationSpeed();
    }
    public void setPower(double power){
        double properTurnPwr = this.m_TurnDeg >= 0 ? Math.abs(power) : -Math.abs(power);
        this.m_TeleOpTask.setDriveRotationSpeed(properTurnPwr);
    }
    public RobotGyro getGyro(){
        return this.m_Gyro;
    }
    public void setGyro(RobotGyro Gyro){
        this.m_Gyro = Gyro;
    }
    @Override
    protected void __startTask() {
        this.getGyro().updateStatus();
        this.m_StartDeg = this.getGyro().getHeading();
        this.m_TeleOpTask.setMotionSystem(super.getMotionSystem());
        this.m_TeleOpTask.startTask();
    }

    @Override
    protected void __taskFinished() {
        this.m_TeleOpTask.stopTask();
    }

    @Override
    public String getTaskDetailString() {
        String result = "TaskType: MotionSystemGyroGuidedTurnTask, ";
        result += "Speed: " + this.getPower() + ", ";
        result += "TurnDeg: " + this.getTurnDeg();
        return result;
    }

    @Override
    public double getTaskProgressRatio() {
        double deltaAng = this.getGyro().getHeading() - m_StartDeg;
        double gyroTurnAng =  (this.getGyro().getHeadingRotationPositiveOrientation() == RobotGyro.HeadingRotationPositiveOrientation.CounterClockwise ? m_TurnDeg : -m_TurnDeg);
        return deltaAng / gyroTurnAng;
    }

    @Override
    public void updateStatus() {
        this.getGyro().updateStatus();
        boolean GyroReversed = this.getGyro().getHeadingRotationPositiveOrientation() == RobotGyro.HeadingRotationPositiveOrientation.CounterClockwise ? false : true;
        double targetPosition = m_StartDeg + (this.getGyro().getHeadingRotationPositiveOrientation() == RobotGyro.HeadingRotationPositiveOrientation.CounterClockwise ? m_TurnDeg : -m_TurnDeg);
        boolean turningCC = this.m_TurnDeg >= 0 ? true : false;
        double currentPosition = this.getGyro().getHeading();
        if(this.m_StartDeg < targetPosition) {
            if (((!GyroReversed) && turningCC) || (GyroReversed && (!turningCC))) {
                if (currentPosition < (this.m_StartDeg - this.getErrorMarginAngle()) || currentPosition > (targetPosition + this.m_ErrorMarginAng)) {
                    this.stopTask();
                }
            }else{ //(((!GyroReversed) && (!turningCC)) || (GyroReversed && turningCC))
                if(currentPosition > (this.m_StartDeg + this.getErrorMarginAngle()) && currentPosition < (targetPosition - this.m_ErrorMarginAng)){
                    this.stopTask();
                }
            }
        }else if(this.m_StartDeg > targetPosition){
            if(((!GyroReversed) && turningCC) || (GyroReversed && (!turningCC))){
                if(currentPosition > (targetPosition + this.m_ErrorMarginAng) && currentPosition < (this.m_StartDeg - this.m_ErrorMarginAng)){
                    this.stopTask();
                }
            }else{ //(((!GyroReversed) && (!turningCC)) || (GyroReversed && turningCC))
                if(currentPosition < (targetPosition - this.m_ErrorMarginAng) || currentPosition > (this.m_StartDeg + this.m_ErrorMarginAng)){
                    this.stopTask();
                }
            }
        }else{ //this.m_StartDeg == targetPosition
            this.stopTask();
        }
    }
    @Override
    public void stopTask(){
        if(this.isBusy() && this.getMotionSystem().isGyroGuidedDriveEnabled()){
            this.getMotionSystem().setGyroGuidedDrivePublicStartingAngle(XYPlaneCalculations.normalizeDeg(this.getMotionSystem().getGyroGuidedDrivePublicStartingAngle() + ((float) this.getTurnDeg())));
        }
        super.stopTask();
    }
}
