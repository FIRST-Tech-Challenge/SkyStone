package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm extends Subsystem {
    DcMotor main;

    int robotHeight;
    int robotLength;
    int blockHeight;
    int foundationHeight;
    int armLength;
    final int armMax = 250;

    public Arm(HardwareMap hardwareMap) {
        main = hardwareMap.dcMotor.get("arm");
        initMotors(new DcMotor[]{main});
    }
    public DcMotor getMain(){
        return main;
    }

    public void setArm(Chassis chassis, int level) {
        double distance;
        double armAngle;
        int heightDif = Math.abs(robotHeight - (blockHeight * level + foundationHeight));
        armAngle = Math.acos((double) heightDif / (double) armLength);
        distance = Math.sqrt((armLength * armLength) - (heightDif * heightDif)) + robotLength;
    }
    public void runArm(int power){

    }


}
