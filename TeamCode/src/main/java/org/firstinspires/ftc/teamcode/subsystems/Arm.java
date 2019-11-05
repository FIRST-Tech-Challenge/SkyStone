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
        initArm();
    }

    public DcMotor getMain() {
        return main;
    }

    public void initArm() {
        reset();
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //This sets it's mode to use a PID loop and input is velocity instead of power
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double setArm(Chassis chassis, int level) {
        double armAngle;
        double distance;
        int heightDif = Math.abs(robotHeight - (blockHeight * level + foundationHeight));
        armAngle = Math.acos((double) heightDif / (double) armLength);
        return distance = Math.sqrt((armLength * armLength) - (heightDif * heightDif)) + robotLength;
    }

    public void runArm(int power) {

    }


}
