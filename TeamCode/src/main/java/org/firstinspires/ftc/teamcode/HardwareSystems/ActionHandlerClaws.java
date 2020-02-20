package org.firstinspires.ftc.teamcode.HardwareSystems;

import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.RobotPoint;

public class ActionHandlerClaws extends ActionHandler{

    private AutoClaws claws;

    public ActionHandlerClaws(AutoClaws claws){
        this.claws = claws;
    }

    @Override
    public void doActions(RobotPoint point){
        claws.setPositions(point.clampPosition, point.hookPosition);
    }
}
