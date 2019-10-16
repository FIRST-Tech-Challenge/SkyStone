package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationGrabber {

    private Servo leftHook;
    private RevTouchSensor leftBlock;

    private static FoundationGrabber instance = null;

    public static synchronized FoundationGrabber getInstance() {
        return instance != null ? instance : (instance = new FoundationGrabber());
    }

    private FoundationGrabber() {}

    public void init(HardwareMap hardwareMap) {
        leftBlock = hardwareMap.get(RevTouchSensor.class, "auto_block_touch_left");
        leftHook = hardwareMap.get(Servo.class, "foundation_hook_left");
        leftHook.scaleRange(0.4, 0.9);
        leftHook.setDirection(Servo.Direction.FORWARD);
    }

    public void setGrabbed(boolean grabbed) {
        leftHook.setPosition(grabbed ? 1 : 0);
    }

    public boolean isGrabbed() {
        return leftBlock.isPressed();
    }
}
