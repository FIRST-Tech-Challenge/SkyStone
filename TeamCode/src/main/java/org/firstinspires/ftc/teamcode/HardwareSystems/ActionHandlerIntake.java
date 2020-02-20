package org.firstinspires.ftc.teamcode.HardwareSystems;

import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.RobotPoint;

public class ActionHandlerIntake extends ActionHandler{

    private Intake intake;
    private Outtake outtake;
    private Extrusion extrusion;

    public ActionHandlerIntake(Intake intake, Outtake outtake, Extrusion extrusion){
        this.intake = intake;
        this.outtake = outtake;
        this.extrusion = extrusion;

    }

    @Override
    public void doActions(RobotPoint point){
        intake.setPower(point.intakePower);
        outtake.setGripperState(point.outtakeClampState);
        outtake.setFlipperState(point.outtakeFlipState);
        extrusion.setTargetPosition(point.liftPosition);

    }

}
