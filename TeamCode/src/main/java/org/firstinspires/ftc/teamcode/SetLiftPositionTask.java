package org.firstinspires.ftc.teamcode;

// Steps: Lift arm, rotate grabber 90 degrees, extend delivery slide, lower block onto skystone
public class SetLiftPositionTask implements RobotControl {

    transient RobotHardware robot;
    transient RobotProfile profile;
    int liftPosition;
    int startPosition;
    long timeDuration;
    double percentComplete;
    transient long timeStart;

    public SetLiftPositionTask(RobotHardware robot, RobotProfile profile, int targetPosition, long timeDuration) {
        this.robot = robot;
        this.profile = profile;
        liftPosition = targetPosition;
        this.timeDuration =  timeDuration;
    }

    public void prepare(){
        timeStart = System.currentTimeMillis();
        startPosition = robot.getEncoderCounts(RobotHardware.EncoderType.LIFT);
    }

    public void execute() {
        if (System.currentTimeMillis() - timeStart < timeDuration){
            percentComplete = (System.currentTimeMillis() - timeStart)/timeDuration;
            robot.setLiftPosition((int)((liftPosition - startPosition)*percentComplete)+startPosition);
        }
        else{
            robot.setLiftPosition(liftPosition);
        }
    }


    public void cleanUp(){

    }

    public boolean isDone() {
        return Math.abs(robot.getEncoderCounts(RobotHardware.EncoderType.LIFT) - liftPosition) < 5;
    }

}
