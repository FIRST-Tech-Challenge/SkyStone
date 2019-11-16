package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm extends Subsystem {
    public DcMotor arm;
    int[] levelAngles = {0,
            -85,
            -136,
            -189,
            -245,
            -311
    };

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.dcMotor.get("arm");
        initMotors(new DcMotor[]{arm});
        initArm();
    }

    public DcMotor getArm() {
        return arm;
    }

    public void initArm() {
        reset();
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setArm(int level) {
        arm.setTargetPosition(levelAngles[level]);
    }

    public void setArmCheck(double error) {

    }

    public void run(int level) {
        setArm(level);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(level!=0){
            arm.setPower(1);
        }
    }
}
