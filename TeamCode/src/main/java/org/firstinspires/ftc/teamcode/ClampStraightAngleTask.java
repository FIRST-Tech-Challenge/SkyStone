package org.firstinspires.ftc.teamcode;

// Steps: Lift arm, rotate grabber 90 degrees, extend delivery slide, lower block onto skystone
public class ClampStraightAngleTask implements RobotControl {

    transient RobotHardware robot;
    transient RobotProfile profile;
    RobotHardware.ClampPosition position;
    transient long timeStart;

    public ClampStraightAngleTask(RobotHardware robot, RobotProfile profile) {
        this.robot = robot;
        this.profile = profile;
    }

    public String toString() {
        return "Clamp to pickup angle";
    }

    public void prepare(){
        timeStart = System.currentTimeMillis();
    }

    public void execute() {
        robot.rotateGrabberOriginPos();
    }

    public void cleanUp(){

    }

    public boolean isDone() {
        return System.currentTimeMillis() > timeStart + 250;
    }

}
