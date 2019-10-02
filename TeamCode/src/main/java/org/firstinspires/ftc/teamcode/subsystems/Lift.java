package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    private DcMotorEx motor;
    private int level;
    private int LEVEL_HEIGHT = 300; // ticks per lift level
    private int heightOffset;
    private int MIN_OFFSET = 0;
    private int MAX_OFFSET = 299;
    private final int MAX_LEVEL = 4;

    private final double P = 1;
    private final double I = 0;
    private final double D = 0.1;
    private final double F = 0.1;

    private static Lift instance = null;

    public static synchronized Lift getInstance() {
        return instance != null ? instance : (instance = new Lift());
    }

    private Lift() {}

    public void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        level = 0;
        heightOffset = 0;
//        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(P, I, D, F));
    }

    public void idle() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(F);
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

    public void setHeightOffset(int offset) {
        if (offset >= MIN_OFFSET && offset <= MAX_OFFSET) {
            heightOffset = offset;
        }
    }

    public int getHeightOffset() {
        return heightOffset;
    }

    public void updatePosition() {
        motor.setTargetPosition(level * LEVEL_HEIGHT + heightOffset);
    }
}
