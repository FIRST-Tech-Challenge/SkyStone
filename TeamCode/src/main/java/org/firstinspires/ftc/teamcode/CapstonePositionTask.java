package org.firstinspires.ftc.teamcode;

// Steps: Lift arm, rotate grabber 90 degrees, extend delivery slide, lower block onto skystone
public class CapstonePositionTask implements RobotControl {

    transient RobotHardware robot;
    transient RobotProfile profile;
    RobotHardware.CapPosition position;
    transient long timeStart;

    public CapstonePositionTask(RobotHardware robot, RobotProfile profile, RobotHardware.CapPosition position) {
        this.robot = robot;
        this.profile = profile;
        this.position = position;
    }

    public String toString() {
        return "Cap to " + position;
    }

    public void prepare(){
        timeStart = System.currentTimeMillis();
    }

    public void execute() {
        robot.setCapStoneServo(position);
    }

    public void cleanUp(){

    }

    public boolean isDone() {
        return System.currentTimeMillis() > timeStart + 250;
    }

}
