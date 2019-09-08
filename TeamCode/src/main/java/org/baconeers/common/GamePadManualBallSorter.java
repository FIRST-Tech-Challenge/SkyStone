package org.baconeers.common;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Operation to assist with Gamepad actions on DCMotors
 */
public class GamePadManualBallSorter extends BaconComponent {


    private final Servo sorterServo;
    private final Gamepad gamepad;

    private double lastPosition = -1.0;

    public GamePadManualBallSorter(BaconOpMode opMode, Gamepad gamepad, Servo sorterservo) {
        super(opMode);

        this.gamepad = gamepad;
        this.sorterServo = sorterservo;

    }

    /**
     * Update servo with latest gamepad state
     */
    public void update() {
        // Only toggle when the button state changes from false to true, ie when the
        // button is pressed down (and not when the button comes back up)
        double position = -1.0;
        boolean pressed = false;
        boolean rightTriggerOn = gamepad.right_trigger > 0;
        boolean leftTriggerOn = gamepad.left_trigger > 0;

        if (leftTriggerOn)
        {
            position = 0.95;

        }else if(rightTriggerOn){
            position = 0.85;
        }


        if (lastPosition != position) {

            sorterServo.setPosition(position);

        }

        lastPosition = position;
    }
}


