package org.baconeers.common;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Operation to assist with Gamepad actions on DCMotors
 */
public class GamePadToggleServo extends BaconComponent {


    private final Servo redservo;
    private final Gamepad gamepad;

    private double lastPosition = -1.0;

    public GamePadToggleServo(BaconOpMode opMode, Gamepad gamepad, Servo redservo) {
        super(opMode);

        this.gamepad = gamepad;
        this.redservo = redservo;

    }

    /**
     * Update servo with latest gamepad state
     */
    public void update() {
        // Only toggle when the button state changes from false to true, ie when the
        // button is pressed down (and not when the button comes back up)
        double position = -1.0;
        boolean pressed = false;
        if (buttonPressed(gamepad, ButtonControl.DPAD_DOWN)) {
            position = 0.2;
            pressed = true;
        } else if (buttonPressed(gamepad, ButtonControl.DPAD_LEFT)) {
            position = 1.0;
            pressed = true;
        } else if (buttonPressed(gamepad, ButtonControl.DPAD_RIGHT)) {
            position = 0;
            pressed = true;
        }


        if (pressed && lastPosition != position) {

            redservo.setPosition(position);

        }

        lastPosition = position;
    }
}


