package org.firstinspires.ftc.teamcode;

public class ClampAngleRotationTask implements RobotControl {

    transient RobotHardware robot;
    transient RobotProfile profile;
    RobotHardware.ClampAnglePosition position;
    transient long timeStart;

    public ClampAngleRotationTask(RobotHardware robot, RobotProfile profile, RobotHardware.ClampAnglePosition position) {
        this.robot = robot;
        this.profile = profile;
        this.position = position;
    }

    public String toString() {
        return "Clamp angle rotates to " + position;
    }

    public void prepare(){
        timeStart = System.currentTimeMillis();
    }

    public void execute() {
        robot.setClampAnglePosition(position);
    }

    public void cleanUp(){

    }

    public boolean isDone() {
        return System.currentTimeMillis() > timeStart + 250;
    }

}
