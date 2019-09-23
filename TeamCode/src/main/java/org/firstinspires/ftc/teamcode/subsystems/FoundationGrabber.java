package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationGrabber {

    private Servo grabber;

    public FoundationGrabber(HardwareMap hardwareMap) {
        grabber = hardwareMap.get(Servo.class, "foundation");
        grabber.scaleRange(0.4, 0.9);
        grabber.setDirection(Servo.Direction.FORWARD);
    }

    public void grab() {
        grabber.setPosition(1);
    }

    public void release() {
        grabber.setPosition(0);
    }

    public boolean isGrabbed() {
        return grabber.getPosition() > 0.5;
    }
}
