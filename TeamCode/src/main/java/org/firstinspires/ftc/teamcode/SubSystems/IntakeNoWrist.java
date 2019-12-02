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
    public Servo grip;
    public Servo grip2;
    public ColorSensor detectSkystone;

    double gripOpenPosition = 0.75; //Value is specific for each grip
    double gripClosePosition = 0.25; //Value is specific for each grip

    //Constructor
    public IntakeNoWrist(HardwareMap hardwareMap) {
        grip = hardwareMap.servo.get("grip");
        grip2 = hardwareMap.servo.get("grip2");
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
        grip.setPosition(gripOpenPosition);
        grip2.setPosition(1.0-gripOpenPosition);
    }

    /**
     * Method to  close Grip
     */
    public void closeGrip(){
        grip.setPosition(gripClosePosition);
        grip2.setPosition(1.0-gripOpenPosition);

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
