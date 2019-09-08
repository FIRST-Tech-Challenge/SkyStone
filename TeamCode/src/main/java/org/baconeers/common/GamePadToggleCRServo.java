package org.baconeers.common;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Operation to assist with Gamepad actions on DCMotors
 */
public class GamePadToggleCRServo extends BaconComponent {

    private final ButtonControl buttonControl;
    private final CRServo crservo;
    private final Gamepad gamepad;
    private final float crservopower;
    private boolean crservoOn = false;
    private boolean lastButtonState = false;
    private final Telemetry.Item item;


    /**
     * Constructor for operation.  Telemetry enabled by default.
     *
     * @param opMode
     * @param gamepad       Gamepad
     * @param motor         DcMotor to operate on
     * @param buttonControl {@link ButtonControl}
     * @param power         power to apply when using gamepad buttons
     * @param showTelemetry  display the power values on the telemetry
     */
    public GamePadToggleCRServo(BaconOpMode opMode, Gamepad gamepad, CRServo crservo, ButtonControl buttonControl, float power, boolean showTelemetry) {
        super(opMode);

        this.gamepad = gamepad;
        this.crservo = crservo;
        this.buttonControl = buttonControl;
        this.crservopower = power;

        if (showTelemetry) {
            item = opMode.telemetry.addData("Control " + buttonControl.name(), 0.0f);
            item.setRetained(true);
        } else {
            item = null;
        }
    }
    public GamePadToggleCRServo(BaconOpMode opMode, Gamepad gamepad, CRServo crservo , ButtonControl buttonControl, float power) {
        this(opMode,gamepad,crservo,buttonControl,power,true);
    }


    /**
     * Update servo with latest gamepad state
     */
    public void update() {
        // Only toggle when the button state changes from false to true, ie when the
        // button is pressed down (and not when the button comes back up)
        boolean pressed = buttonPressed(gamepad, buttonControl);
        if (pressed && lastButtonState != pressed) {
            crservoOn = !crservoOn;
            float power = crservoOn ? crservopower : 0.0f;
            crservo.setPower(power);
            if (item != null) {
                item.setValue(power);
            }
            getOpMode().telemetry.log().add("%s crservo: %.2f", buttonControl.name(), power);
        }
        lastButtonState = pressed;
    }


}
