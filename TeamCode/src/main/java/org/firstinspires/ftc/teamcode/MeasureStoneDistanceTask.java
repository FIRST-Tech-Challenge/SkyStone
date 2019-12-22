package org.firstinspires.ftc.teamcode;

// Use distance sensor to measure the stone distance, and calculate the optimum slider count
public class MeasureStoneDistanceTask implements RobotControl {
    boolean completed;
    long optimalSlidePos;
    transient RobotHardware robot;
    transient RobotProfile profile;

    public MeasureStoneDistanceTask(RobotHardware robot, RobotProfile profile) {
        this.robot = robot;
        this.profile = profile;
    }

    public String toString() {
        return "StoneDistMeasure";
    }

    public void prepare(){
        completed = false;
    }

    public void execute() {
        RobotProfile.HardwareSpec spec = profile.hardwareSpec;
        double distInCM = robot.getRightDistance();
        optimalSlidePos = Math.min((int)(spec.sliderCountPerCM * (distInCM - spec.sliderOutMinCM)) + spec.sliderOutMin,
                                    spec.sliderOutMax);
        Logger.logFile("Stone distance: " + distInCM + " optimalCount:" + optimalSlidePos);
        completed = true;
    }

    public void cleanUp(){
    }

    public boolean isDone() {
        return completed;
    }

    public long getOptimalSliderPosition() {
        return optimalSlidePos;
    }
}
