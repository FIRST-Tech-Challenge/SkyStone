package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationGrabber {

    private Servo grabber;

    private static FoundationGrabber instance = null;

    public static FoundationGrabber getInstance(HardwareMap hardwareMap) {
        return instance != null ? instance : (instance = new FoundationGrabber(hardwareMap));
    }

    private FoundationGrabber(HardwareMap hardwareMap) {
        grabber = hardwareMap.get(Servo.class, "foundation");
        grabber.scaleRange(0.4, 0.9);
        grabber.setDirection(Servo.Direction.FORWARD);
    }

    public void setGrabbed(boolean grabbed) {
        grabber.setPosition(grabbed ? 1 : 0);
    }

    public boolean isGrabbed() {
        return grabber.getPosition() > 0.5;
    }
}
