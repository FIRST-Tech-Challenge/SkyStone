package org.firstinspires.ftc.teamcode;

public class IntakeControlTask implements RobotControl {

    transient RobotHardware robot;
    transient RobotProfile profile;
    RobotHardware.IntakeDirection direction;
    transient long timeStart;

    public IntakeControlTask(RobotHardware robot, RobotProfile profile, RobotHardware.IntakeDirection direction) {
        this.robot = robot;
        this.profile = profile;
        this.direction = direction;
    }

    public String toString() {
        return "Intake to " + direction;
    }

    public void prepare(){
        timeStart = System.currentTimeMillis();
    }

    public void execute() {
        robot.setIntakeDirection(direction);
    }

    public void cleanUp(){

    }

    public boolean isDone() {
        return System.currentTimeMillis() > timeStart + 100;
    }

}
