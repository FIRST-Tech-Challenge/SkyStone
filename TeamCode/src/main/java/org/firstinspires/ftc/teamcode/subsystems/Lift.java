package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.ButtonAndEncoderData;

public class Lift {

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private RevColorSensorV3 blockDetector;
    private final double MAX_NO_BLOCK_DIST_IN = 1;
    private boolean hadBlock;
    private RevTouchSensor bottomLimit;
    private int level;
    private final int LEVEL_HEIGHT = 300; // ticks per lift level
    private final int MAX_LEVEL = 4;

    private static Lift instance = null;

    public static synchronized Lift getInstance() {
        return instance != null ? instance : (instance = new Lift());
    }

    private Lift() {}

    public void init(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotorEx.class, "liftRight/odometerX");
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor = hardwareMap.get(DcMotorEx.class, "liftLeft");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        double aParam = 315;
        double bInvParam = 0.605;
        double cParam = 169.7;
        blockDetector = new RevColorSensorV3(
                hardwareMap.get(RevColorSensorV3.class, "liftBlockColor").getDeviceClient()
        ) {
            @Override
            protected double inFromOptical(int rawOptical) {
                return Math.pow((rawOptical - cParam) / aParam, -bInvParam);
            }
        };

        bottomLimit = hardwareMap.get(RevTouchSensor.class, "liftLimitSwitch");

        level = 0;

        hadBlock = false;
        setNoBlockPid();
        updatePid();
        wasIdling = false;
        wasStill = true;
    }

    private boolean wasIdling;
    private boolean wasStill;

    public void idle() {
        if (ButtonAndEncoderData.getLatest().isPressed(bottomLimit)) {
            if (!wasIdling) {
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftMotor.setPower(0.1);
                rightMotor.setPower(0.1);
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                wasIdling = true;
            }
        } else {
            if (!wasStill) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                wasStill = true;
            }
        }
    }

    public void zero() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        if (level != 0) {
            wasStill = false;
        }
        wasIdling = false;
        leftMotor.setTargetPosition(level * LEVEL_HEIGHT);
    }

    public void updateRightMotor() {
        if (level != 0) {
            rightMotor.setPower(Robot.getInstance().controlHub.getMotorVoltagePercent(leftMotor));
            wasStill = false;
        } else if (!wasStill) {
            rightMotor.setPower(0);
            wasStill = true;
        }
    }

    private void setNoBlockPid() {
        PIDFCoefficients pidfVelocity = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfPosition = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        pidfVelocity.f = 0.1;
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfVelocity);
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfPosition);
    }

    private void setBlockPid() {
        PIDFCoefficients pidfVelocity = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfPosition = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        pidfVelocity.f = 0.1;
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfVelocity);
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfPosition);
    }

    public boolean hasBlock() {
        return blockDetector.getDistance(DistanceUnit.INCH) < MAX_NO_BLOCK_DIST_IN;
    }

    private void updatePid() {
        boolean hasBlock = hasBlock();
        if (hadBlock && !hasBlock) {
            setNoBlockPid();
            hadBlock = false;
        }
        if (!hadBlock && hasBlock) {
            setBlockPid();
            hadBlock = true;
        }
    }
}
