package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.ButtonAndEncoderData;

public class FoundationGrabber {

    private Servo leftHook;
    private RevTouchSensor leftBlock;
    private RevColorSensorV3 leftEye;
    private Servo rightHook;
    private RevTouchSensor rightBlock;
    private RevColorSensorV3 rightEye;

    private RevTouchSensor frontTouch;
    private RevTouchSensor sideTouch;

    private final double LEFT_GRABBED_POSITION = 0.2;
    private final double LEFT_UNGRABBED_POSITION = 0.8;
    private final double RIGHT_GRABBED_POSITION = 0.2;
    private final double RIGHT_UNGRABBED_POSITION = 0.8;

    private final double RED_TO_BLUE_THRESHOLD = 1.5;
    private final double GREEN_TO_BLUE_THRESHOLD = 2.25;
    private final double MAX_STONE_VISIBLE_DISTANCE_IN = 5;

    private static FoundationGrabber instance = null;

    public static synchronized FoundationGrabber getInstance() {
        return instance != null ? instance : (instance = new FoundationGrabber());
    }

    private FoundationGrabber() {}

    public void init(HardwareMap hardwareMap) {
        leftHook = hardwareMap.get(Servo.class, "foundationHookLeft");
        leftHook.scaleRange(LEFT_UNGRABBED_POSITION, LEFT_GRABBED_POSITION);
        leftHook.setDirection(Servo.Direction.FORWARD);

        leftBlock = hardwareMap.get(RevTouchSensor.class, "autoBlockTouchLeft");

        double aParam = 315;
        double bInvParam = 0.605;
        double cParam = 169.7;
        leftEye = new RevColorSensorV3(
                hardwareMap.get(RevColorSensorV3.class, "ssColorLeft").getDeviceClient()
        ) {
            @Override
            protected double inFromOptical(int rawOptical) {
                return Math.pow((rawOptical - cParam) / aParam, -bInvParam);
            }
        };

        rightHook = hardwareMap.get(Servo.class, "foundationHookRight");
        rightHook.scaleRange(RIGHT_UNGRABBED_POSITION, RIGHT_GRABBED_POSITION);
        rightHook.setDirection(Servo.Direction.FORWARD);

        rightBlock = hardwareMap.get(RevTouchSensor.class, "autoBlockTouchRight");

        rightEye = new RevColorSensorV3(
                hardwareMap.get(RevColorSensorV3.class, "ssColorRight").getDeviceClient()
        ) {
            @Override
            protected double inFromOptical(int rawOptical) {
                return Math.pow((rawOptical - cParam) / aParam, -bInvParam);
            }
        };

        frontTouch = hardwareMap.get(RevTouchSensor.class, "foundationTouchFront");
        sideTouch = hardwareMap.get(RevTouchSensor.class, "foundationTouchSide");
    }

    public void setGrabbed(Hook hook, boolean grabbed) {
        switch (hook) {
            case LEFT:
                leftHook.setPosition(1);
                break;
            case RIGHT:
                rightHook.setPosition(1);
                break;
            case BOTH:
                leftHook.setPosition(1);
                rightHook.setPosition(1);
                break;
            default:
                throw new IllegalArgumentException("Unknown or invalid hook " + hook + " provided.");
        }
    }

    public boolean isGrabbedOnButton(Hook hook) {
        switch (hook) {
            case LEFT:
                return ButtonAndEncoderData.getLatest().isPressed(leftBlock);
            case RIGHT:
                return ButtonAndEncoderData.getLatest().isPressed(rightBlock);
            case BOTH:
                return ButtonAndEncoderData.getLatest().isPressed(leftBlock)
                        && ButtonAndEncoderData.getLatest().isPressed(rightBlock);
            case EITHER:
                return ButtonAndEncoderData.getLatest().isPressed(leftBlock)
                        || ButtonAndEncoderData.getLatest().isPressed(rightBlock);
            default:
                throw new IllegalArgumentException("Unknown or invalid hook " + hook + " provided.");
        }
    }

    public SeenObject getView(Hook hook, double distance) {
        if (distance > MAX_STONE_VISIBLE_DISTANCE_IN) {
            return SeenObject.NOTHING;
        }
        switch (hook) {
            case LEFT:
                NormalizedRGBA rgbaL = leftEye.getNormalizedColors();
                return rgbaL.red > rgbaL.blue * RED_TO_BLUE_THRESHOLD && rgbaL.green > rgbaL.blue * GREEN_TO_BLUE_THRESHOLD
                    ? SeenObject.STONE : SeenObject.SKYSTONE;
            case RIGHT:
                NormalizedRGBA rgbaR = rightEye.getNormalizedColors();
                return rgbaR.red > rgbaR.blue * RED_TO_BLUE_THRESHOLD && rgbaR.green > rgbaR.blue * GREEN_TO_BLUE_THRESHOLD
                        ? SeenObject.STONE : SeenObject.SKYSTONE;
            default:
                throw new IllegalArgumentException("Unknown or invalid hook " + hook + " provided.");
        }
    }

    public double getDistance(Hook hook) {
        switch (hook) {
            case LEFT:
                return leftEye.getDistance(DistanceUnit.INCH);
            case RIGHT:
                return rightEye.getDistance(DistanceUnit.INCH);
            default:
                throw new IllegalArgumentException("Unknown or invalid hook " + hook + " provided.");
        }
    }

    public boolean frontTouchingFoundation() {
        return frontTouch.isPressed();
    }

    public boolean sideTouchingFoundation() {
        return sideTouch.isPressed();
    }

    public enum Hook {
        LEFT,
        RIGHT,
        BOTH,
        EITHER
    }

    public enum SeenObject {
        STONE,
        SKYSTONE,
        NOTHING
    }
}
