package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    private DcMotorEx motor;
    private int level;
    private int LEVEL_HEIGHT = 300; // ticks per lift level
    private final int MAX_LEVEL = 4;

    private static Lift instance = null;

    public static synchronized Lift getInstance() {
        return instance != null ? instance : (instance = new Lift());
    }

    private Lift() {}

    public void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "lift_right");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        level = 0;
        // set pid
    }

    public void idle() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.1);
    }

    public void zero() {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setLevel(int level) {
        this.level = level;
        if (level > MAX_LEVEL) {
            this.level = MAX_LEVEL;
        }
        if (level < 0) {
            this.level = 0;
        }
    }

    public int getLevel() {
        return level;
    }

    public void moveUp() {
        level++;
        if (level > MAX_LEVEL) {
            level = MAX_LEVEL;
        }
    }

    public void moveDown() {
        level--;
        if (level < 0) {
            level = 0;
        }
    }

    public void updatePosition() {
        motor.setTargetPosition(level * LEVEL_HEIGHT);
    }
}
