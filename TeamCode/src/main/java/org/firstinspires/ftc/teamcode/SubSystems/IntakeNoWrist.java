package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Definition of Intake Mechanism without wrist.
 * Intake has :
 *      1 ServoMotor for Wrist motion that operates in 3 modes -
 *          INITIAL, VERITICAL_BLOCK, HORIZONTAL_BLOCK)
 *      1 Linear Actutator which a Servomotor to open and close Grip mechanism
 *      1 Color Sensor for identifying skystone and blocks
 *
 * @IntakeMethods : toggleGrip()
 * @IntakeAutoMethods : openGrip()
 * @IntakeAutoMethods : closeGrip()
 * @IntakeAutoMethods : detectSkystoneColorSensorIsYellow()
 * @IntakeAutoMethods : detectSkystoneColorSensorIsBlack()
 */

/**
 * Class Definition
 */
public class IntakeNoWrist {
    //public Servo grip;
    //public Servo grip2;

    public Servo left_grip;
    public Servo right_grip;

    public ColorSensor detectSkystone;

    final static double GRIP_HOME = 0.0;
    final static double GRIP_MIN_RANGE = 0;
    final static double GRIP_MAX_RANGE = 1;
    final static double GRIP_OPEN_LEFT = 0.24;
    final static double GRIP_OPEN_RIGHT = 0.74;
    final static double GRIP_SPEED=0.01;

    final static double GRIP_CLOSE_LEFT = 0.73;
    final static double GRIP_CLOSE_RIGHT= 0.23;

    double gripOpenPosition = 0.75; //Value is specific for each grip
    double gripClosePosition = 0.25; //Value is specific for each grip

    double grip2OpenPosition = 1 - gripOpenPosition;
    double grip2ClosePosition = 1 - gripClosePosition;



    //Constructor
    public IntakeNoWrist(HardwareMap hardwareMap) {
        //grip = hardwareMap.servo.get("grip");
        //grip2 = hardwareMap.servo.get("grip2");

        left_grip = hardwareMap.servo.get("left_grip");
        right_grip = hardwareMap.servo.get("right_grip");
        left_grip.scaleRange(GRIP_MIN_RANGE,GRIP_MAX_RANGE);
        right_grip.scaleRange(GRIP_MIN_RANGE,GRIP_MAX_RANGE);

        detectSkystone = hardwareMap.get(ColorSensor.class, "detect_skystone");
        initIntake();
    }

    //#TOBEFILLED Consider initializing position?
    public void initIntake() {
        detectSkystone.enableLed(false);
    }

    /**
     * Method to open Grip
     */
    public void openGrip(){
        //grip2.setPosition(gripOpenPosition);
        //grip.setPosition(grip2OpenPosition);
        left_grip.setPosition(GRIP_OPEN_LEFT);
        right_grip.setPosition(GRIP_OPEN_RIGHT);
    }

    /**
     * Method to  close Grip
     */
    public void closeGrip(){

        left_grip.setPosition(GRIP_CLOSE_LEFT);
        right_grip.setPosition(GRIP_CLOSE_RIGHT);
        //grip2.setPosition(gripClosePosition);
        //grip.setPosition(grip2ClosePosition);

    }

    /**
     * Method to open and close grip based on switching from current state.
     */
    public void toggleGrip() {
        if (left_grip.getPosition() == GRIP_OPEN_LEFT) {
            closeGrip();
        } else {
            openGrip();
        }
    }



    /**
     * Method to set Intake Color sensor Off
     */
    public void resetIntake() {
        detectSkystone.enableLed(false);
        openGrip();
    }

    /**
     * Method to check for detectSkystone Color Sensor to sense Yellow block or Skystone black color
     * Used in Autonomous mode to identify skystone and regular block.
     * @return true if Yellow is detected, else false
     */
    public boolean detectSkystoneColorSensorIsYellow() {
        //Logic to detect Yellow R>150 G>150 B<100
        detectSkystone.enableLed(true);
        if (detectSkystone.red()>150 && detectSkystone.green()>150 && detectSkystone.blue()<100 && detectSkystone.alpha()>60) {
            detectSkystone.enableLed(false);
            return true;
        } else {
            detectSkystone.enableLed(false);
            return false;
        }
    }

    /**
     * Method to check for detectSkystone Color Sensor to sense Yellow block or Skystone black color
     * Used in Autonomous mode to identify skystone and regular block.
     * @return true if Black is detected, else false
     */
    public boolean detectSkystoneColorSensorIsBlack() {
        //Logic to detect Black R<64 G<64 B<64
        detectSkystone.enableLed(true);
        if (detectSkystone.red()<64 && detectSkystone.green()<64 && detectSkystone.blue()<64) {
            detectSkystone.enableLed(false);
            return true;
        } else {
            detectSkystone.enableLed(false);
            return false;
        }
    }

    // #TOBEDELETED
// all below are not finished yet, will do saturday, Ian.
/*
    public void setwrist(double position) {
        wrist.setPosition(position);
    }
    public void setgrip(double position) {
        grip.setPosition(position);
    }
// values need to be tested
    public void setWristToHighPosition() {
        wrist.setPosition(wristHighPosition);
    }
// values need to be tested
    public void setWristToLowPosition() {
        wrist.setPosition(wristLowPosition);
    }
    // values need to be tested
    public void setgripToHighPosition() {
        grip.setPosition(gripHighPosition);
    }
// values need to be tested
    public void setgripToLowPosition() {
        grip.setPosition(gripLowPosition);
    }
*/

}