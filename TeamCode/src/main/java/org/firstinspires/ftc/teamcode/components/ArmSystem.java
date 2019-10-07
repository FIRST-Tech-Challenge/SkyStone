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
    private final double WRIST_HOME = 0;
    private final double ELBOW_HOME = 0;
    private final double PIVOT_HOME = 0;
    private final double GRIPPER_OPEN = 0;
    private final double GRIPPER_CLOSE = 0.5; // I think? TODO: Figure out overheating issueo

    public enum Position {
        POSITION_HOME, POSITION_WEST, POSITION_SOUTH, POSITION_EAST, POSITION_NORTH
    }
    /*
     If the robot is at the bottom of the screen, and X is the block:

     XO
     XO  <--- Position A

     OO
     XX  <--- Position B

     OX
     OX  <--- Position C

     XX
     OO  <--- Position D

     Probably should be controlled by the D pad or something.
     */

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

    public void movePivot(double pos) {
        // Uncomment this line when build team implements the pivot
        // elbow.setPosition(pos);
    }

    public void increaseGripper(double pos) {
        if (gripper.getPosition() + pos <= 1 && gripper.getPosition() + pos >= 0) {
            gripper.setPosition(gripper.getPosition());
        }
    }

    public void increaseWrist(double pos) {
        if (wrist.getPosition() + pos <= 1 && wrist.getPosition() + pos >= 0) {
            wrist.setPosition(wrist.getPosition());
        }
    }

    public void increaseElbow(double pos) {
        if (elbow.getPosition() + pos <= 1 && elbow.getPosition() + pos >= 0) {
            elbow.setPosition(elbow.getPosition());
        }
    }

    public void increasePivot(double pos) {
        if (pivot.getPosition() + pos <= 1 && pivot.getPosition() + pos >= 0) {
            //gripper.setPosition(gripper.getPosition());
        }
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

    public double getPivot() {
        return 0;
    }

    // Moves the arm to the "home state" - the grabber is open, right above the block in the intake.
    // The values of the servos in the home state can be set by editing the final variables.
    public void goHome() {
        openGripper();
        moveWrist(WRIST_HOME);
        moveElbow(ELBOW_HOME);
        movePivot(PIVOT_HOME);
    }

    public void openGripper() {
        moveGripper(GRIPPER_OPEN);
    }

    public void closeGripper() {
        moveGripper(GRIPPER_CLOSE);
    }

    public void movePresetPosition(Position pos) {
        switch(pos) {
            case POSITION_HOME:
                openGripper();
                moveWrist(0);
                moveElbow(0);
                movePivot(0);
            case POSITION_NORTH:
                moveWrist(0);
                moveElbow(0);
                movePivot(0);
            case POSITION_EAST:
                moveWrist(0);
                moveElbow(0);
                movePivot(0);
            case POSITION_WEST:
                moveWrist(0);
                moveElbow(0);
                movePivot(0);
            case POSITION_SOUTH:
                moveWrist(0);
                moveElbow(0);
                movePivot(0);
        }
    }
}
