package org.firstinspires.ftc.teamcode.components;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import  com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    private final double GRIPPER_OPEN = 0.7;
    private final double GRIPPER_CLOSE = 0.3;
    private int origin;

    // This is in block positions, not ticks
    private int targetHeight;
    private final int distanceConstant = 500; // used for calculating motor speed

    // Use these so we can change it easily if the motor is put on backwards
    private final DcMotor.Direction UP = DcMotor.Direction.REVERSE;
    private final DcMotor.Direction DOWN = DcMotor.Direction.FORWARD;

    // For the queueing system, which we need to figure out how the drivers feel about it
    private Position QueuedPosition;
    private int QueuedHeight;

    // These fields are used only for calibration. Don't touch them outside of that method.
    private boolean calibrated = false;
    private enum Direction {
        UP, DOWN;
        private static Direction reverse(Direction direction){
            return direction == UP ? DOWN : UP;
        }

        private static DcMotorSimple.Direction motorDirection(Direction direction){
            return direction == UP ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        }
    };

    private Direction direction;

    // Don't change this unless in calibrate() or init(), is read in the calculateHeight method
    private int calibrationDistance;

    // This can actually be more, like 5000, but we're not going to stack that high
    // for the first comp and the servo wires aren't long enough yet
    private final int MAX_HEIGHT = 3000;
    private final int INCREMENT_HEIGHT = 564; // how much the ticks increase when a block is added
    private final int START_HEIGHT = 366; // Height of the foundation

    private double SERVO_SPEED;
    private final double SERVO_TOLERANCE = 0.02;
    private double pivotTarget = 0.09;
    private double elbowTarget = 0.09;
    private double wristTarget = 0.62;


    // Set to true when we're in the process of going home
    private boolean homing = false;

    private boolean fastMode;

    // I know in terms of style points these should be private and just have getters and setters but
    // I want to make them easily incrementable
    public Position queuedPosition;

    public int queuedHeight;

    public enum Position {
        POSITION_HOME, POSITION_WEST, POSITION_SOUTH,
        POSITION_EAST, POSITION_NORTH, POSITION_CAPSTONE
    }

    private EnumMap<Position, double[]> positionEnumMap;

    public enum ServoNames {
        GRIPPER, WRIST, ELBOW, PIVOT
    }

    public static String toReturn = "";
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
     */
    public ArmSystem(EnumMap<ServoNames, Servo> servos, DcMotor slider, DigitalChannel limitSwitch,
                     boolean calibrate) {
        this.gripper = servos.get(ServoNames.GRIPPER);
        this.wrist = servos.get(ServoNames.WRIST);
        this.elbow = servos.get(ServoNames.ELBOW);
        this.pivot = servos.get(ServoNames.PIVOT);
        this.slider = slider;
        this.limitSwitch = limitSwitch;
        if (calibrate) {
            calibrate();
        } else {
            this.calibrationDistance = slider.getCurrentPosition();
        }
        this.direction = Direction.UP;
        this.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // We map Position enums to an array of their servo values.
        // We might want to move to these values in different parts of code,
        // for example if we want to turn on / off fastmode, that can be done easily.
        // Double values ordered Pivot, elbow, wrist.
        this.positionEnumMap = new EnumMap<Position, double[]>(Position.class);
        positionEnumMap.put(Position.POSITION_NORTH, new double[] {0.99, 0.58, 0.05});
        positionEnumMap.put(Position.POSITION_EAST, new double[] {0.99, 0.22, 0.37});
        positionEnumMap.put(Position.POSITION_WEST, new double[] {0.99, 0.22, 0.72});
        positionEnumMap.put(Position.POSITION_SOUTH, new double[] {0.99, 0.22, 0.37});
        positionEnumMap.put(Position.POSITION_HOME, new double[] {0.19, 0.15, 0.79});
        positionEnumMap.put(Position.POSITION_CAPSTONE, new double[] {0.42, 0.31, 0.75});
    }

    /*
        Use this method in teleop.
        Set assist to true if you want to use driver assist. Same w/ fastmode.
        Each of these values should just be the raw button data (including gripper, which is
        a toggle.)
        The method variables are for keeping track of the buttons, so our driver class can just give
        the button input.
        It's a string so we can debug to telemetry, and use queuing if we implement it.
     */
    private boolean m_gripper, m_up, m_down = false;
    public String run(boolean home, boolean capstone, boolean west, boolean east, boolean north, boolean south,
                      boolean up, boolean down, boolean gripperButton, boolean assist,
                      double sliderSpeed, double armSpeed, boolean fastMode) {

        this.fastMode = fastMode;

        if (homing) {
            goHome();
            // We don't want Teddy to screw with the arm while it's going home and have it break
            return "";
        }
        this.SERVO_SPEED = armSpeed;

        if (west) {
            toReturn += "Moving west!";
            movePresetPosition(Position.POSITION_WEST);
        } else if (east) {
            movePresetPosition(Position.POSITION_EAST);
            toReturn += "Moving east!";
        } else if (south) {
            movePresetPosition(Position.POSITION_SOUTH);
        } else if (north) {
            movePresetPosition(Position.POSITION_NORTH);
        }  else if (capstone) {
            movePresetPosition(Position.POSITION_CAPSTONE);
        } else if (home) {
            homing = true;
        }


        if (assist) {
            if (up && !m_up) {
                setSliderHeight(++targetHeight);
                m_up = true;
            } else if (!up) {
                m_up = false;
            }

            if (down && !m_down) {
                setSliderHeight(--targetHeight);
                m_down = true;
            } else if (!down) {
                m_down = false;
            }
            updateHeight(sliderSpeed);
            toReturn += "Down: " + down;
        } else {
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (up) {
                slider.setDirection(Direction.motorDirection(Direction.UP));
            } else if (down) {
                slider.setDirection(Direction.motorDirection(Direction.DOWN));
            }
            slider.setPower(sliderSpeed);
        }

        if (gripperButton && !m_gripper) {
            toggleGripper();
            m_gripper = true;
        } else if (!gripperButton) {
            m_gripper = false;
        }
        String temp = toReturn;
        toReturn = "";
        return temp;
    }

    /*
    // You can thank me later Adrian
    // Returns true when done, although this should probably be changed because idrk how state
    // machines work
    private enum autoState {
        GO_HOME, CLOSE_GRIPPER, GO_UP, GO_DOWN, DROP
    }
    private autoState m_currState = autoState.CLOSE_GRIPPER;
    public boolean runAuto(Position position) {
        switch(m_currState) {
            case GO_HOME:
                m_currState = autoState.OPEN_GRIPPER;
                break;
            case OPEN_GRIPPER:
                m_currState = autoState.GO_UP
        }
    }
    */

    // These are public for debugging purposes
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

    // Moves the slider up to one block high, moves the gripper to the home position, and then moves
    // back down so we can fit under the bridge.
    private Direction m_homeDirection = Direction.UP;
    private void goHome() {
        if (m_homeDirection == Direction.UP) {
            int diff = getSliderPos() - calculateHeight(0);
            if (Math.abs(diff) < 100) {

            }
            setSliderHeight(1);
            if (getSliderPos() == calculateHeight(1)) {
                // We know we can set fastmode to true here because we always want it to go fast
                // to the home position
                movePresetPosition(Position.POSITION_HOME);
                openGripper();
                m_homeDirection = Direction.DOWN;
            }
        } else {
            // This should bring it to the lowest possible state, although it might get weird
            // with the reversing directions so we should figure this out. If we just move the
            // slider down, which way even is down when the motors reverse all the time?
            // Should we even have the reversing direction?
            setSliderHeight(-1);
            if (getSliderPos() == calculateHeight(-1)) {
                m_homeDirection = Direction.UP;
                homing = false; // We're done!
            }
        }
        updateHeight(1);

    }

    private void openGripper() {
        gripper.setPosition(GRIPPER_OPEN);
    }

    private void closeGripper() {
        gripper.setPosition(GRIPPER_CLOSE);
    }

    private void toggleGripper() {
        if (Math.abs(gripper.getPosition() - GRIPPER_CLOSE)
                < Math.abs(gripper.getPosition() - GRIPPER_OPEN)) {
            // If we're in here, the gripper is closer to it's closed position
            openGripper();
        } else {
            closeGripper();
        }
    }

    private void movePresetPosition(Position pos) {
        double[] posArray = positionEnumMap.get(pos);
        if (fastMode) {
            pivot.setPosition(posArray[0]);
            elbow.setPosition(posArray[1]);
            wrist.setPosition(posArray[2]);
        } else {
            pivotTarget = posArray[0];
            elbowTarget = posArray[1];
            wristTarget = posArray[2];
        }
    }

    private void setQueuedPosition(Position position, int height) {
        this.QueuedPosition = position;
    }

    // Still requires updateHeight() to be called, just like the setSliderHeight method.
    private void go() {
        //this.movePresetPosition(queuedPosition);
        setSliderHeight(queuedHeight);
    }


    /*
    Takes in the slack in the linear slide. Used to calibrate the encoder.
    Must be called every iteration of init_loop.
    Initially, the slider will be resting on the switch, resulting in
    limit.Switch.getState() == false
     */
    private void calibrate() {

        // The following code has a high chance of causing a timeout in init.
        // The commented code doesn't timeout but it also doesn't work.
        // Everybody seems to agree that we don't really need calibration anyway so it should be
        // fine, just don't run this code. I'm keeping it here because we might want it later,
        // although calling it in init() or start() will take more time in the round which might
        // cause problems.
        // TODO: Make this use init_loop so it doesn't timeout.
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

    private void stop() {
        slider.setPower(0);
    }

    // Pos should be the # of blocks high it should be
    private void setSliderHeight(int pos) {
        targetHeight = pos;
        slider.setTargetPosition(calculateHeight(targetHeight));
        slider.setDirection(Direction.motorDirection(Direction.UP));
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Log.d(TAG, "Set target height to" + calculateHeight(targetHeight));
    }

    // Little helper method for setSliderHeight
    private int calculateHeight(int pos) {
        return START_HEIGHT + calibrationDistance + (pos * INCREMENT_HEIGHT);
    }

    // Should be called every loop
    private void updateHeight(double speed) {
        slider.setPower(speed);
        slider.setTargetPosition(calculateHeight(targetHeight));
        if (!fastMode) {
            updateServo(elbow, elbowTarget);
            toReturn += elbowTarget + "\n";
            updateServo(pivot, pivotTarget);
            toReturn += pivotTarget + "\n";
            updateServo(wrist, wristTarget);
            toReturn += wristTarget + "\n";
        }
    }

    private void updateServo(Servo servo, double pos) {
        if (servo.getPosition() - pos > SERVO_TOLERANCE) {
            servo.setPosition(servo.getPosition() - SERVO_SPEED);
        } else if (servo.getPosition() - pos < -SERVO_TOLERANCE) {
            servo.setPosition(servo.getPosition() + SERVO_SPEED);
        } else if (servo.getPosition() != pos) {
            servo.setPosition(pos);
        }
    }

    // Use these for debugging
    public boolean getSwitchState() {
        return limitSwitch.getState();
    }
    public int getSliderPos() { return slider.getCurrentPosition(); }
}
