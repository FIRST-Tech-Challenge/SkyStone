package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Definition of Intake Mechanism.
 * Intake has :
 *      1 ServoMotor for Wrist motion that operates in 3 modes -
 *          INITIAL, VERITICAL_BLOCK, HORIZONTAL_BLOCK)
 *      1 Linear Actutator which a Servomotor to open and close Grip mechanism
 *      1 Color Sensor for identifying skystone and blocks
 *
 * @IntakeMethods : moveWristToInitialPosition()
 * @IntakeMethods : moveWristToHorizontalPosition()
 * @IntakeMethods : moveWristToVerticalPosition()
 * @IntakeTeleOpMethods : moveWristUp()
 * @IntakeTeleOpMethods : moveWristDown()
 * @IntakeMethods : toggleGrip()
 * @IntakeAutoMethods : openGrip()
 * @IntakeAutoMethods : closeGrip()
 * @IntakeAutoMethods : detectSkystoneColorSensorIsYellow()
 * @IntakeAutoMethods : detectSkystoneColorSensorIsBlack()
 */

/**
 * Class Definition
 */
public class Intake{
    public Servo wrist;
    public Servo grip;
    public ColorSensor detectSkystone;

    //initialize limit positions
    double wristClosePosition = 0.2; // = 0.7 #TOBEDEFINED
    double wristVerticalPosition = 0.5; // = 0.5 #TOBEDEFINED
    double wristHorizontalPosition = 1.0 ; // = 1.0#TOBEDEFINED
    double wristCurrentPosition;

    double gripOpenPosition = 0.75; //Value is specific for each grip
    double gripClosePosition = 0.25; //Value is specific for each grip

    //Constructor
    public Intake(HardwareMap hardwareMap) {
        wrist = hardwareMap.servo.get("wrist");
        grip = hardwareMap.servo.get("grip");

        detectSkystone = hardwareMap.get(ColorSensor.class, "detect_skystone");
        initIntake();
    }

    //#TOBEFILLED Consider initializing position?
    public void initIntake() {
        detectSkystone.enableLed(false);
        moveWristToHorizontal();
    }

    /**
     * Method to open Grip
     */
    public void openGrip(){
        grip.setPosition(gripOpenPosition);
    }

    /**
     * Method to  close Grip
     */
    public void closeGrip(){
        grip.setPosition(gripClosePosition);
    }

    /**
     * Method to open and close grip based on switching from current state.
     */
    public void toggleGrip() {
        if (grip.getPosition() == gripOpenPosition) {
            closeGrip();
        } else {
            openGrip();
        }
    }

    /**
     * Method to move wrist to Initial position
     */
    public void moveWristToClose(){
        wrist.setPosition(wristClosePosition);
    }

    /**
     * Method to move wrist to Vertical position
     */
    public void moveWristToVertical(){
        wrist.setPosition(wristVerticalPosition);
    }

    /**
     * Method to move wrist to Horizontal position
     */
    public void moveWristToHorizontal(){
        wrist.setPosition(wristHorizontalPosition);
    }

    /**
     * Method to manage state of wrist and move based on dpad_up input
     */
    public void moveWristUp(){
        wristCurrentPosition = wrist.getPosition();
        //if currently in close position, move to vertical position (tolerance 0f +/- 0.05)
        if (wristCurrentPosition == wristClosePosition) {
            moveWristToVertical();
        }
        //if currently in vertical position, move to horizontal position (tolerance 0f +/- 0.05)
        if (wristCurrentPosition == wristVerticalPosition) {
            moveWristToHorizontal();
        }
        //moveWristToHorizontal();

    }

    /**
     * Method to manage state of wrist and move based on dpad_down input
     */
    public void moveWristDown(){
        wristCurrentPosition = wrist.getPosition();
        //if currently in horizontal position, move to vertical position
        if (wristCurrentPosition == wristHorizontalPosition) {
            moveWristToVertical();
        }
        //if currently in vertical position, move to close position
        if (wristCurrentPosition == wristVerticalPosition) {
            moveWristToClose();
        }
        //moveWristToClose();

    }


    /**
     * Method to set Intake Color sensor Off
     */
    public void resetIntake() {
        detectSkystone.enableLed(false);
        moveWristToHorizontal();
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
