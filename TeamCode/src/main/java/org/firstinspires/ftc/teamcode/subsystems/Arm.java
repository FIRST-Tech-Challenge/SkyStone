package org.firstinspires.ftc.teamcode.subsystems;

public class Arm {
    int robotHeight;
    int robotLength;
    int blockHeight;
    int foundationHeight;
    int armLength;

    public void setArm(int level) {
        double distance;
        double armAngle;
        int heightDif = Math.abs(robotHeight-(blockHeight*level+foundationHeight));
        armAngle = Math.acos((double)heightDif/(double)armLength);
        distance = Math.sqrt((armLength*armLength)-(heightDif*heightDif))+robotLength;

    }
}
