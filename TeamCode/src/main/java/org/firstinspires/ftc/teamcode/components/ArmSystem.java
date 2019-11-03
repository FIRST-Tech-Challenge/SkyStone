package org.firstinspires.ftc.teamcode.components;
import android.util.Log;

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
    private DigitalChannel limitSwitch; // true is unpressed, false is pressed
    private final double WRIST_HOME = 0;
    private final double ELBOW_HOME = 0;
    private final double PIVOT_HOME = 0;
    private final double GRIPPER_OPEN = 0.5;
    private final double GRIPPER_CLOSE = 0.08;
    private int origin;
    public int targetHeight;
    private final int distanceConstant = 1000; // used for calculating motor speed

    // Use these so we can change it easily if the motor is put on backwards
    private final DcMotor.Direction UP = DcMotor.Direction.REVERSE;
    private final DcMotor.Direction DOWN = DcMotor.Direction.FORWARD;
    protected Position QueuedPosition;
    protected int QueuedHeight;

    // These fields are used only for calibration. Don't touch them outside of that method.
    private boolean calibrated = false;
    public enum Direction {
        UP, DOWN;
        private static Direction reverse(Direction direction){
            return direction == UP ? DOWN : UP;
        }

        private static DcMotorSimple.Direction motorDirection(Direction direction){
            return direction == UP ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        }
    };

    private Direction direction;

    // Don't change this unless in calibrate(), is read in the calculateHeight method
    private int calibrationDistance = 0;

    // This can actually be more, like 5000, but we're not going to stack that high
    // for the first comp and the servo wires aren't long enough yet
    public final int MAX_HEIGHT = 3000;
    public final int INCREMENT_HEIGHT = 564; // how much the ticks increase when a block is added
    public final int START_HEIGHT = 366; // Height of the foundation

    // I know in terms of style points these should be private and just have getters and setters but
    // I want to make them easily incrementable
    public Position queuedPosition;
    public int queuedHeight;

    public enum Position {
        POSITION_HOME, POSITION_WEST, POSITION_SOUTH, POSITION_EAST, POSITION_NORTH
    }

    public enum ServoNames {
        GRIPPER, WRIST, ELBOW, PIVOT
    }

    public static final String TAG = "ArmSystem"; // for debugging

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
        this.limitSwitch = limitSwitch;
        this.queuedPosition = Position.POSITION_HOME;
        // this.direction = Direction.UP;
        // this.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Create an ArmSystem object without servos, used for testing just the slider
    public ArmSystem(DcMotor slider, DigitalChannel limitSwitch) {
        this.slider = slider;
        this.limitSwitch = limitSwitch;
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
    // It requires the slider to be attached, so it can go over the latch.
    // The values of the servos in the home state can be set by editing the final variables.
    public void goHome() {
        openGripper();
        setSliderHeight(1);
        moveWrist(0.06);
        moveElbow(0.68);
        movePivot(0.83);
        setSliderHeight(0);
    }

    public void openGripper() {
        moveGripper(GRIPPER_OPEN);
    }

    public void closeGripper() {
        moveGripper(GRIPPER_CLOSE);
    }

    public void toggleGripper() {
        if (Math.abs(gripper.getPosition() - GRIPPER_CLOSE)
                < Math.abs(gripper.getPosition() - GRIPPER_OPEN)) {
            // If we're in here, the gripper is closer to it's closed position
            openGripper();
        } else {
            closeGripper();
        }
    }

    public void movePresetPosition(Position pos) {
        switch(pos) {
            case POSITION_HOME:
                goHome();
                break;
            case POSITION_NORTH:
                // TODO: Find north pos with new motor
                moveWrist(0.88);
                moveElbow(0.9);
                movePivot(0.1);
                break;
            case POSITION_EAST:
                moveWrist(0.62);
                moveElbow(0.12);
                movePivot(0.1);
                break;
            case POSITION_WEST:
                moveWrist(0.17);
                moveElbow(0.6);
                movePivot(0.1);
                break;
            case POSITION_SOUTH:
                moveWrist(0.62);
                moveElbow(0.6);
                movePivot(0.1);
                break;
        }
    }

    public void setQueuedPosition(Position position, int height) {
        this.QueuedPosition = position;
    }

    // Still requires updateHeight() to be called, just like the setSliderHeight method.
    public void go() {
        this.movePresetPosition(queuedPosition);
        setSliderHeight(queuedHeight);
    }


    /*
    Takes in the slack in the linear slide. Used to calibrate the encoder.
    Must be called every iteration of init_loop.
    Initially, the slider will be resting on the switch, resulting in
    limit.Switch.getState() == false
     */
    public void calibrate() {
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setDirection(Direction.motorDirection(direction));
        slider.setPower(0.1);

        // going up
        for( int i = 0; i < 1111; i++) {
            // go up
            slider.setDirection(Direction.motorDirection(Direction.UP));
            slider.setPower(0.2);
            if(limitSwitch.getState()) {
                break;
            }
            try {
                Thread.sleep(2);
            } catch (Exception e) {

            }

        }
        slider.setPower(0);
        //going down
        for( int i = 0; i < 1111; i++) {
            // go up
            slider.setDirection(Direction.motorDirection(Direction.DOWN));
            slider.setPower(0.2);
            if(!limitSwitch.getState()) {
                break;
            }
            try {
                Thread.sleep(2);
            } catch (Exception e) {

            }
        }
        calibrated = true;
        this.calibrationDistance = slider.getCurrentPosition();
        slider.setPower(0);
        /*
        // At rest, pre-int, this will be false, i.e. the limit switch IS pressed.
        if (limitSwitch.getState()) {
            // Limit switch has been released, cord is under tension.
            // With cord under tension, lower until switch is depressed.
            direction = Direction.DOWN;
        }

        Log.d(TAG,"Calibrating");
        Log.d(TAG, "Direction: " + direction);
        Log.d(TAG, "Switch State: " + limitSwitch.getState());
        // If we're going down and we hit the switch, then we're done
        if (direction == Direction.DOWN && !limitSwitch.getState()) {
            slider.setPower(0);
            calibrated = true;
            calibrationDistance = slider.getCurrentPosition();
            slider.setDirection(Direction.motorDirection(Direction.UP));
            setSliderHeight(0);
        }

         */
    }

    public boolean isCalibrated() {
        return calibrated;
    }

    public void stop() {
        slider.setPower(0);
    }

    // Pos should be the # of blocks high it should be
    public void setSliderHeight(int pos) {
        targetHeight = calculateHeight(pos);
        if (targetHeight > MAX_HEIGHT) throw new IllegalArgumentException();
        slider.setTargetPosition(targetHeight);
        slider.setDirection(Direction.motorDirection(Direction.UP));
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Log.d(TAG, "Set target height to" + calculateHeight(pos));
    }

    // Little helper method for setSliderHeight
    private int calculateHeight(int pos) {
        return START_HEIGHT + calibrationDistance + (pos * INCREMENT_HEIGHT);
    }

    // Should be called every loop
    public void updateHeight() {
        slider.setPower(1);
        slider.setTargetPosition(targetHeight);
    }

    // Use these for debugging
    public boolean getSwitchState() {
        return limitSwitch.getState();
    }
    public int getSliderPos() { return slider.getCurrentPosition(); }
}
