package org.firstinspires.ftc.teamcode;

public class HookPositionTask implements RobotControl {

    transient RobotHardware robot;
    transient RobotProfile profile;
    RobotHardware.HookPosition position;
    transient long timeStart;

    public HookPositionTask(RobotHardware robot, RobotProfile profile, RobotHardware.HookPosition position) {
        this.robot = robot;
        this.profile = profile;
        this.position = position;
    }

    public String toString() {
        return "Hook Position " + position;
    }


    public void prepare(){
        timeStart = System.currentTimeMillis();
    }

    public void execute() {
        robot.setHookPosition(position);
    }

    public void cleanUp(){

    }

    public boolean isDone() {
        return System.currentTimeMillis() > timeStart + 250;
    }

}
