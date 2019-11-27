package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Definition of Intake Mechanism.
 * Intake has :
 *      1 ServoMotor for Wrist motion that operates in 3 modes -
 *          INITIAL, VERITICAL_BLOCK, HORIZONTAL_BLODK)
 *      1 Linear Actutator which a Servomotor to open and close Grip mechanism
 *      1 Color Sensor for identifying skystone and blocks
 *
 * @IntakeMethods : moveWristToInitialPosition()
 * @IntakeMethods : moveWristToHorizontalPosition()
 * @IntakeMethods : moveWristToVerticalPosition()
 * @IntakeMethods : openGrip()
 * @IntakeMethods : closeGrip()
 * @IntakeMethods : toggleGrip()
 * @IntakeMethods : detectSkystoneColorSensorIsYellow
 * @IntakeMethods : detectSkystoneColorSensorIsBlack
 */

/**
 * Class Definition
 */
public class Intake{
    Servo wrist;
    Servo grip;
    ColorSensor detectSkystone;

    //initialize limit positions
    double wristInitialPosition; // = 0.7 #TOBEDEFINED
    double wristHorizontalPosition; // = 0.5#TOBEDEFINED
    double wristVerticalPosition; // = 1.0 #TOBEDEFINED

    double gripOpenPosition = 0.75;
    double gripClosePosition = 0.25;

    //Constructor
    public Intake(HardwareMap hardwareMap) {
        //wrist = hardwareMap.servo.get("wrist");
        //grip = hardwareMap.servo.get("grip");
        //detectSkystone = hardwareMap.colorSensor.get("detectSkystone");
        initIntake();
    }

    //#TOBEFILLED Consider initializing position?
    public void initIntake() {

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
        wrist.setPosition(wristInitialPosition);
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
     * Method to set Intake Color sensor Off
     */
    public void resetIntake() {
        detectSkystone.enableLed(false);
        moveWristToHorizontal();
        openGrip();
    }

    /**
     * Method to set the ColorSensor to be enabled or disabled
     * @param colorSensorEnabled
     */
    public void setDetectSkystoneColorSensordEnabled(boolean colorSensorEnabled){
        detectSkystone.enableLed(colorSensorEnabled);
    }

    /**
     * Method to check for detectSkystone Color Sensor to sense Yellow block or Skystone black color
     * Used in Autonomous mode to identify skystone and regular block.
     * @return
     */
    public boolean detectSkystoneColorSensorIsYellow() {
        if (detectSkystone.red()>127 && detectSkystone.green()>127 && detectSkystone.blue()<127) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Method to check for detectSkystone Color Sensor to sense Yellow block or Skystone black color
     * Used in Autonomous mode to identify skystone and regular block.
     * @return
     */
    public boolean detectSkystoneColorSensorIsBlack() {
        if (detectSkystone.red()<127 && detectSkystone.green()<127 && detectSkystone.blue()<127) {
            return true;
        } else {
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
