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
    }

    public void execute() {
        if (this.hookPosition== RobotHardware.HookPosition.HOOK_ON && System.currentTimeMillis() - timeStart < timeDuration){
            robot.rotateGrabberOriginPos();
        }
        else if(this.hookPosition == RobotHardware.HookPosition.HOOK_OFF){
            robot.rotateGrabberOriginPos();
        }
    }


    public void cleanUp(){

    }

    public boolean isDone() {
        return System.currentTimeMillis() > timeStart + 250;
    }
}
