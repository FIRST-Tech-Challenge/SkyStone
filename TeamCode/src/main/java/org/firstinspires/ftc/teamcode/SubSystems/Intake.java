package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Definition of Intake Mechanism.
 * Intake has :
 *      1 ServoMotor for Wrist motion that operates in 3 modes -
 *          INITIAL, VERITICAL_BLOCK, HORIZONTAL_BLOCK)
 *      1 Linear Actutator which a Servomotor to open and close Grip mechanism
 *      1 Color Sensor for identifying skystone and blocks
 *
 *      Experiment to determing Stone vs Skystone logic based on color / distance sensor
 *      detectSkystone on Skystone Edge : R248 G434 B224 Al:894 Hue: 50331904
 *
 *      detectSkystone on Skystone distance 0" : D0.83 A4098 R1242 G2207 B1146
 *      detectSkystone on Skystone distance 1" : D1.92 A1000 R257 G477 B280
 *      detectSkystone on Skystone distance 2" : D2.53 A700 R185 G339 B165
 *      detectSkystone on Skystone distance 3" : D3.22 A615 R162 G299 B164
 *      detectSkystone on Skystone distance 4" : D3.53 A549 R159 G275 B150
 *
 *      detectSkystone on Stone distance 0" : D0.43 A20807 R772 G1048 B1919
 *      detectSkystone on Stone distance 1" : D1.56 A2197 R796 G1165 B282
 *      detectSkystone on Stone distance 2" : D2.10 A1325 R464 G693 B205
 *      detectSkystone on Stone distance 3" : D2.67 A910 R309 G472 B165
 *      detectSkystone on Stone distance 4" : D3.00 A730 R237 G373 B150
 *
 *      detectSkystone on Mat not near anything : D4.05 A1480 R135 G240 B126
 *
 *      Logic :
 *      Distance goes below D2.75" - stone is nearby - Could be Skystone around 3" away or Stone around 3.3" away
 *      R>250 G>400 means stone, else Skystone
 *      If stone, go forward 2" - drop arm
 *
 *
 *
 * Logic :
 *     Distance goes below D2.75" - stone is nearby - Could be Skystone around 3" away or Stone around 3.3" away
 *     R>250 G>400 means stone, else Skystone
 *     If stone, go forward 2" - drop arm
 *
 * @IntakeMethods : moveWristToClose()
 * @IntakeMethods : moveWristToHorizontal()
 * @IntakeMethods : moveWristToVertical()
 * @IntakeTeleOpMethods : moveWristUp()
 * @IntakeTeleOpMethods : moveWristDown()
 * @IntakeMethods : toggleGrip()
 * @IntakeAutoMethods : openGrip()
 * @IntakeAutoMethods : closeGrip()
 * @IntakeAutoMethods : detectSkystoneAndType()
 * @IntakeAutoMethods : detectSkystoneColor()
 * @IntakeAutoMethods : detectSkystoneDistance()
 *
 * @return :skystoneDetected
 * @return : stoneDetected
 */
public class Intake {
    public Servo wrist;
    public Servo grip;
    public ColorSensor detectSkystoneColor;
    public DistanceSensor detectSkystoneDistance;

    //Wrist position values on servo motor from close to Vertocal to midPositon1 to midPsition2 to Horizontal
    public int wristCurrentPosition;
    public double[] wristPosition = {
            0.2,  //closePosition
            0.5,  //VerticalPosition
            0.66, //MidPosition1
            0.83, //MidPosition2
            1.0,  //HorizontalPosition
    };
    public boolean skystoneDetected;
    public boolean stoneDetected;

    //Open and close Position for the grip linear actuator
    public double gripOpenPosition = 0.75;
    public double gripClosePosition = 0.25;

    //Constructor
    public Intake(HardwareMap hardwareMap) {
        wrist = hardwareMap.servo.get("wrist");
        grip = hardwareMap.servo.get("grip");

        detectSkystoneColor = hardwareMap.get(ColorSensor.class, "detect_skystone");
        detectSkystoneDistance = hardwareMap.get(DistanceSensor.class, "detect_skystone");
        //initIntake();
    }

    //On Start Move wrist to horizontal position
    public void initIntake() {
        detectSkystoneColor.enableLed(false);
        moveWristToHorizontal();
        openGrip();
    }

    /**
     * Method to open Grip
     */
    public void openGrip() {
        grip.setPosition(gripOpenPosition);
    }

    /**
     * Method to  close Grip
     */
    public void closeGrip() {
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
    public void moveWristToClose() {
        wrist.setPosition(wristPosition[0]);//close position = 0.2
        wristCurrentPosition = 0;
    }

    /**
     * Method to move wrist to Vertical position
     */
    public void moveWristToVertical() {
        wrist.setPosition(wristPosition[1]); //vertical position = 0.5
        wristCurrentPosition = 1;
    }

    /**
     * Method to move wrist to Horizontal position
     */
    public void moveWristToHorizontal() {
        wrist.setPosition(wristPosition[4]); //Horizontal position = 1.0
        wristCurrentPosition = 4;
    }

    /**
     * Method to manage state of wrist and move based on dpad_up input
     * from Close to Vertical to MidPosition1 to MidPosition2 to Horizontal
     */

    public void moveWristUp() {
        if (wristCurrentPosition < 4) {
            wrist.setPosition(wristPosition[wristCurrentPosition + 1]); //move to next higher position
            wristCurrentPosition++;
        }
    }

    /**
     * Method to manage state of wrist and move based on dpad_down input
     * from Horizontal to MidPosition2 to MidPosition1 to Vertical to Close
     */
    public void moveWristDown() {
        if (wristCurrentPosition > 0) {
            wrist.setPosition(wristPosition[wristCurrentPosition - 1]); //move to next higher position
            wristCurrentPosition--;
        }
    }

    /**
     * Method to set Intake Color sensor Off
     */
    public void resetIntake() {
        detectSkystoneColor.enableLed(false);
        moveWristToHorizontal();
        openGrip();
    }

    /**
     * Method to check for detectSkystone Color Sensor to sense Yellow Stone or Skystone color
     * Used in Autonomous mode to identify skystone and regular block.
     * Uses distance and color measurements to make determination.
     * Logic :
     *     Distance goes below D2.75" - stone is nearby - Could be Skystone around 3" away or Stone around 3.3" away
     *     R>250 G>400 means stone, else Skystone
     *    If stone, go forward 2" - drop arm
     *
     * @return true if Yellow is detected, else false
     */
    public boolean detectSkytoneAndType() {
        stoneDetected = false;
        skystoneDetected = false;
        detectSkystoneColor.enableLed(true);
        if (detectSkystoneDistance.getDistance(DistanceUnit.INCH) < 2.75){
            if (detectSkystoneColor.red()>250 && detectSkystoneColor.green() > 400){
                //Stone is detected, but is not skystone
                stoneDetected = true;
                skystoneDetected = false;
                detectSkystoneColor.enableLed(false);
                return true ; //Stone is detected
            } else {
                //Stone is detected and is Skystone is detected
                stoneDetected = true;
                skystoneDetected = true;
                detectSkystoneColor.enableLed(false);
                return true ; //Stone is detected
            }
        }
        return false; // No stone is detected
    }
}