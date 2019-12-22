package org.firstinspires.ftc.teamcode;

public class MeasureAndSlideTask implements RobotControl {
    transient RobotHardware robot;
    transient RobotProfile profile;
    int sliderPosition;
    int startPosition;
    long timeDuration;
    double percentComplete;
    transient long timeStart;

    public MeasureAndSlideTask(RobotHardware robot, RobotProfile profile, long timeDuration) {
        this.robot = robot;
        this.profile = profile;
        this.timeDuration =  timeDuration;
    }

    public String toString() {
        return "Measure then Slide";
    }

    public void prepare(){
        timeStart = System.currentTimeMillis();
        startPosition = robot.getEncoderCounts(RobotHardware.EncoderType.SLIDER);

        RobotProfile.HardwareSpec spec = profile.hardwareSpec;
        double finalDistInCM;
        double distInCM1 = robot.getRightDistance();
        try{ Thread.sleep(50); } catch(Exception e){}
        double distInCM2 = robot.getRightDistance();
        try{ Thread.sleep(50); } catch(Exception e){}
        double distInCM3 = robot.getRightDistance();
        if((distInCM2 < distInCM3 && distInCM2 > distInCM1) || (distInCM2 < distInCM1 && distInCM2 > distInCM3)){
            finalDistInCM = distInCM2;
        } else if((distInCM1 < distInCM3 && distInCM1 > distInCM2) || (distInCM1 < distInCM2 && distInCM1 > distInCM3)){
            finalDistInCM = distInCM1;
        } else{
            finalDistInCM = distInCM3;
        }
        // TEMPORARY
        // TEMPORARY
        // TEMPORARY
        finalDistInCM += 3; // When reading from Skystone (black), the distance is always 3cm less by sensor

        sliderPosition = Math.min((int)(spec.sliderCountPerCM * (finalDistInCM - spec.sliderOutMinCM)) + spec.sliderOutMin,
                spec.sliderOutMax);
        Logger.logFile("Stone distance: " + finalDistInCM + " optimalCount:" + sliderPosition);
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
        return Math.abs(robot.getEncoderCounts(RobotHardware.EncoderType.SLIDER) - sliderPosition) < 5;
    }
}
