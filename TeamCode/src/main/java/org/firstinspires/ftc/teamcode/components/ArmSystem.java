package org.firstinspires.ftc.teamcode.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import  com.qualcomm.robotcore.hardware.Servo;

/*
    This class controls everything related to the arm, including driver assist features.

    IMPORTANT: When working on this class (and arm stuff in general),
    keep the servo names consistent: (from closest to the block to farthest)
        - Gripper
        - Wrist
        - Elbow
        - Pivot
 */
public class ArmSystem {
    private Servo gripper;
    private Servo wrist;
    private Servo elbow;
    private Servo pivot; // Not yet implemented by build team, ignore until we have it
    protected HardwareMap hardwareMap;

    public ArmSystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.gripper = hardwareMap.get(Servo.class, "gripper");
        this.wrist = hardwareMap.get(Servo.class, "wrist");
        this.elbow = hardwareMap.get(Servo.class, "elbow");
        // this.pivot = hardwareMap.get(Servo.class, "pivot");
    }

    public void moveGripper(double pos) {
        gripper.setPosition(pos);
    }

    public void moveWrist(double pos) {
        wrist.setPosition(pos);
    }

    public void moveElbow(double pos) {
        elbow.setPosition(pos);
    }

    public double getGripper() {
        return gripper.getPosition();
    }

    public double getWrist() {
        return wrist.getPosition();
    }

    public double getElbow() {
        return elbow.getPosition();
    }

}
