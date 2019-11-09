package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm extends Subsystem {
    DcMotor main;

    double robotHeight;
    double robotLength;
    double blockHeight;
    double foundationHeight;
    double armLength;
    final int armMax = 250;
    int[] levelAngles;

    public Arm(HardwareMap hardwareMap) {
        main = hardwareMap.dcMotor.get("arm");
        initMotors(new DcMotor[]{main});
        initArm();
    }

    public DcMotor getMain() {
        return main;
    }

    public void initArm() {
        reset();
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    public void setArm(int level) {
        double heightDif = getHeightDiffrence(level);
        double armAngle = Math.acos((double) heightDif / (double) armLength);
        main.setTargetPosition(Math.max((int) armAngle * 1440, armMax));
        //main.setTargetPosition(levelAngles[level]);
    }

    public void setArmCheck(double error){

    }

    public void run(int level) {
        setArm(level);
        main.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getDistanceForLevel(int level) {
        double heightDif = getHeightDiffrence(level);
        return Math.sqrt((armLength * armLength) - (heightDif * heightDif)) + robotLength;
    }
    public double getHeightDiffrence(int level){
        return Math.abs(robotHeight - (blockHeight * level + foundationHeight));
    }
}
