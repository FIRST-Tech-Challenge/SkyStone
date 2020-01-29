package org.firstinspires.ftc.teamcode;

// Steps: Lift arm, rotate grabber 90 degrees, extend delivery slide, lower block onto skystone
public class SetSliderPositionTask implements RobotControl {

    transient RobotHardware robot;
    transient RobotProfile profile;
    int sliderPosition;
    int startPosition;
    long timeDuration;
    double percentComplete;
    transient long timeStart;

    public SetSliderPositionTask(RobotHardware robot, RobotProfile profile, int targetPosition, long timeDuration) {
        this.robot = robot;
        this.profile = profile;
        sliderPosition = targetPosition;
        this.timeDuration =  timeDuration;
    }

    public String toString() {
        return "SliderPos to: " + sliderPosition;
    }

    public void prepare(){
        timeStart = System.currentTimeMillis();
        startPosition = robot.getEncoderCounts(RobotHardware.EncoderType.SLIDER);
    }

    public void execute() {
        if (System.currentTimeMillis() - timeStart < timeDuration){
            percentComplete = (System.currentTimeMillis() - timeStart)/timeDuration;
            robot.setSliderPosition((int)((sliderPosition - startPosition)*percentComplete)+startPosition);
        }
        else{
            robot.setSliderPosition(sliderPosition);
        }
    }


    public void cleanUp(){

    }

    public boolean isDone() {
        return Math.abs(robot.getEncoderCounts(RobotHardware.EncoderType.SLIDER) - sliderPosition) < 10;
    }

}
