package org.firstinspires.ftc.teamcode;

public class RotateGrabberTask implements RobotControl {
    transient RobotHardware robot;
    transient RobotProfile profile;
    RobotHardware.HookPosition hookPosition;
    int startPosition;
    long timeDuration;
    double percentComplete;
    transient long timeStart;

    public RotateGrabberTask(RobotHardware robot, RobotProfile profile, RobotHardware.HookPosition hookPosition, long timeDuration) {
        this.robot = robot;
        this.profile = profile;
        this.hookPosition = hookPosition;
        this.timeDuration =  timeDuration;
    }

    public void prepare(){
        timeStart = System.currentTimeMillis();
        //startPosition = robot.getEncoderCounts(RobotHardware.EncoderType.LIFT);
    }

    public void execute() {
        if (this.hookPosition== RobotHardware.HookPosition.HOOK_ON && System.currentTimeMillis() - timeStart < timeDuration){
            robot.rotateGrabberForPickup();
            //percentComplete = (System.currentTimeMillis() - timeStart)/timeDuration;
            //robot.setLiftPosition((int)((liftPosition - startPosition)*percentComplete)+startPosition);
        }
        else if(this.hookPosition == RobotHardware.HookPosition.HOOK_OFF){
            //robot.setLiftPosition(liftPosition);
            robot.rotateGrabberOriginPos();
        }
    }


    public void cleanUp(){

    }

    public boolean isDone() {
        //return Math.abs(robot.getEncoderCounts(RobotHardware.EncoderType.LIFT) - liftPosition) < 5;
        return System.currentTimeMillis() > timeStart + 250;
    }
}
