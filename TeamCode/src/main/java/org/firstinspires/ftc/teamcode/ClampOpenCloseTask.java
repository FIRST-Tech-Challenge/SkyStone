package org.firstinspires.ftc.teamcode;

public class ClampOpenCloseTask implements RobotControl {

    transient RobotHardware robot;
    transient RobotProfile profile;
    RobotHardware.ClampPosition position;
    transient long timeStart;

    public ClampOpenCloseTask(RobotHardware robot, RobotProfile profile, RobotHardware.ClampPosition position) {
        this.robot = robot;
        this.profile = profile;
        this.position = position;
    }

    public String toString() {
        return "Clamp to " + position;
    }

    public void prepare(){
        timeStart = System.currentTimeMillis();
    }

    public void execute() {
        robot.setClampPosition(position);
    }

    public void cleanUp(){

    }

    public boolean isDone() {
        return System.currentTimeMillis() > timeStart + 250;
    }

}
