package org.firstinspires.ftc.teamcode;

// Steps: Lift arm, rotate grabber 90 degrees, extend delivery slide, lower block onto skystone
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
