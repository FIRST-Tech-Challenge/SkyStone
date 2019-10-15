package org.firstinspires.ftc.teamcode.components;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import  com.qualcomm.robotcore.hardware.Servo;
import java.util.EnumMap;

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
    private Servo pivot;
    private DcMotor slider;
    private DigitalChannel limitSwitch; // DigitalChannel is just fancy talk for a switch
    private final double WRIST_HOME = 0;
    private final double ELBOW_HOME = 0;
    private final double PIVOT_HOME = 0;
    private final double GRIPPER_OPEN = 0.47;
    private final double GRIPPER_CLOSE = 0;
    private int origin;
    private int targetHeight;
    private final int distanceConstant = 1000; // used for calculating motor speed

    // Use these so we can change it easily if the motor is put on backwards
    private final DcMotor.Direction FORWARD = DcMotor.Direction.FORWARD;
    private final DcMotor.Direction REVERSE = DcMotor.Direction.REVERSE;

    private int[] positions = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000};
    protected Position QueuedPosition;

    public enum Position {
        POSITION_HOME, POSITION_WEST, POSITION_SOUTH, POSITION_EAST, POSITION_NORTH
    }

    public enum ServoNames {
        GRIPPER, WRIST, ELBOW, PIVOT
    }

    /*
     If the robot is at the bottom of the screen, and X is the block:

     XO
     XO  <--- Position west

     OO
     XX  <--- Position south

     OX
     OX  <--- Position east

     XX
     OO  <--- Position north

     Probably should be controlled by the D pad or something.
     */

    public ArmSystem(EnumMap<ServoNames, Servo> servos, DcMotor slider, DigitalChannel limitSwitch) {
        this.gripper = servos.get(ServoNames.GRIPPER);
        this.wrist = servos.get(ServoNames.WRIST);
        this.elbow = servos.get(ServoNames.ELBOW);
        this.pivot = servos.get(ServoNames.PIVOT);
        this.slider = slider;

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
        pivot.setPosition(pos);
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
            gripper.setPosition(gripper.getPosition());
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
                moveWrist(0);
                moveElbow(0.35);
                movePivot(0.86);
                break;
            case POSITION_NORTH:
                moveWrist(0.88);
                moveElbow(0.9);
                movePivot(0.1);
                break;
            case POSITION_EAST:
                moveWrist(0.55);
                moveElbow(0.9);
                movePivot(0.1);
                break;
            case POSITION_WEST:
                moveWrist(0.1);
                moveElbow(0.45);
                movePivot(0.1);
                break;
            case POSITION_SOUTH:
                moveWrist(0.55);
                moveElbow(0.45);
                movePivot(0.1);
                break;
        }
    }

    public void setQueuedPosition(Position position) {
        this.QueuedPosition = position;
    }

    public void go() {
        this.movePresetPosition(QueuedPosition);
    }

    // Moves the slider down until it hits the limit switch. Used to callibrate the encoder.
    public void callibrate() {
        slider.setDirection(REVERSE);
        while (!limitSwitch.getState()) {
            slider.setPower(0.1);
        }
        slider.setPower(0);
        slider.setDirection(FORWARD);
        // move up slightly to fix tension on the string
        while (limitSwitch.getState()) {
            slider.setPower(0.1);
        }
        this.origin = slider.getCurrentPosition();
    }

    // Pos should be the # of blocks high it should be
    // IMPORTANT - MUST BE SET EVERY LOOP OF AN OPMODE
    public void setSliderHeight(int pos) {
        if (pos > 8) throw new IllegalArgumentException();
        targetHeight = pos;
    }

    public void updateHeight() {
        /*
        if (slider.getCurrentPosition() > targetHeight) {
            slider.setDirection(REVERSE);
            slider.setPower((slider.getCurrentPosition() - targetHeight) / );
        } else {
            slider.setDirection(FORWARD);
        }

        slider.setPower(0.1);

         */
        slider.setPower(1);
        slider.setTargetPosition(targetHeight);
    }


}
