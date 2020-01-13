package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Definition of Intake Mechanism. <BR>
 * Intake has : <BR>
 *      1 ServoMotor for Wrist motion that operates in 3 modes - <BR>
 *          INITIAL, VERITICAL_BLOCK, HORIZONTAL_BLOCK) <BR>
 *      2 ServoMotor which a Servomotor to open and close Grip mechanism <BR>
 *
 * @IntakeMethods : moveWristToClose()
 * @IntakeMethods : moveWristToHorizontal()
 * @IntakeMethods : moveWristToVertical()
 * @IntakeTeleOpMethods : moveWristUp()
 * @IntakeTeleOpMethods : moveWristDown()
 * @IntakeMethods : toggleGrip()
 * @IntakeAutoMethods : openGrip()
 * @IntakeAutoMethods : closeGrip()
 *
 */
public class Intake {
    public Servo wrist;
    public Servo left_grip;
    public Servo right_grip;

    final static double GRIP_MIN_RANGE = 0;
    final static double GRIP_MAX_RANGE = 1;

    public double GRIP_INIT_LEFT = 0.7;
    public double GRIP_INIT_RIGHT = 0.3;

    public  double GRIP_OPEN_LEFT = 0.5;
    public  double GRIP_OPEN_RIGHT = 0.5;

    public double GRIP_CLOSE_LEFT = 0.20;
    public double GRIP_CLOSE_RIGHT= 0.80;

    public int grip_state=0; // 0-INIT, 1-OPEN, 2-CLOSE

    //Wrist position values on servo motor from close to Vertocal to midPositon1 to midPsition2 to Horizontal
    public int wristCurrentPosition;
    public double[] wristPosition = {
            0.62,  //closePosition
            0.70, //0.62,  //VerticalPosition
            0.80, //0.66, //MidPosition1
            0.90, //0.83, //MidPosition2
            1.00  //HorizontalPosition
    };

    //Constructor
    public Intake(HardwareMap hardwareMap) {
        wrist = hardwareMap.servo.get("wrist");
        left_grip = hardwareMap.servo.get("left_grip");
        right_grip = hardwareMap.servo.get("right_grip");
        left_grip.scaleRange(GRIP_MIN_RANGE,GRIP_MAX_RANGE);
        right_grip.scaleRange(GRIP_MIN_RANGE,GRIP_MAX_RANGE);
    }

    //On Start Move wrist to horizontal position
    public void initIntake() {
        moveWristToHorizontal();
        initGrip();
    }

    /**
     * On Start for TeleOp, Dont move intake to protect arm from locking intake, manual action required
     */
        public void initIntakeTeleOp() {
        //moveWristToHorizontal();
        initGrip();
    }

    /**
     * Method to init Grip
     */
    public void initGrip() {
        left_grip.setPosition(GRIP_INIT_LEFT);
        right_grip.setPosition(GRIP_INIT_RIGHT);
        grip_state = 0;//INIT
    }

    /**
     * Method to open Grip
     */
    public void openGrip() {
        left_grip.setPosition(GRIP_OPEN_LEFT);
        right_grip.setPosition(GRIP_OPEN_RIGHT);
        grip_state = 1;//OPEN
    }

    /**
     * Method to  close Grip
     */
    public void closeGrip() {
        left_grip.setPosition(GRIP_CLOSE_LEFT);
        right_grip.setPosition(GRIP_CLOSE_RIGHT);
        grip_state = 2;//CLOSE
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
        //detectSkystoneColor.enableLed(false);
        moveWristToHorizontal();
        openGrip();
    }

}