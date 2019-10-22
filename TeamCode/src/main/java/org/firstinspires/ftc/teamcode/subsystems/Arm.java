package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;



public class Arm {
    int robotHeight;
    int robotLength;
    int blockHeight;
    int foundationHeight;
    int armLength;

    public Arm(HardwareMap hardwareMap) {

    }

    public void setArm(Chassis chassis, int level) {
        double distance;
        double armAngle;
        int heightDif = Math.abs(robotHeight - (blockHeight * level + foundationHeight));
        armAngle = Math.acos((double) heightDif / (double) armLength);
        distance = Math.sqrt((armLength * armLength) - (heightDif * heightDif)) + robotLength;
    }

}
