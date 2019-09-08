package org.baconeers.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Operation to assist with Gamepad actions on DCMotors
 */
public class GamePadToggleMotorWithRevers extends BaconComponent {

    private final ButtonControl buttonControl;
    private final ButtonControl buttonControl2;
    private final DcMotor motor;
    private final Gamepad gamepad;
    private float motorPower;
    private boolean motorOn = false;
    private boolean reverseOn = false;
    private boolean lastButtonState = false;
    private boolean lastButtonState2 = false;
    private final Telemetry.Item item;
    private double lastpower;
    private boolean showtelemetry = false;


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
    public GamePadToggleMotorWithRevers(BaconOpMode opMode, Gamepad gamepad, DcMotor motor, ButtonControl buttonControl, ButtonControl buttonControl2, float power, boolean showTelemetry) {
        super(opMode);

        this.gamepad = gamepad;
        this.motor = motor;
        this.buttonControl = buttonControl;
        this.buttonControl2 = buttonControl2;
        this.motorPower = power;

        if (showTelemetry) {
            item = opMode.telemetry.addData("Control " + buttonControl.name(), 0.0f);
            item.setRetained(true);
        } else {
            item = null;
        }
    }
    public GamePadToggleMotorWithRevers(BaconOpMode opMode, Gamepad gamepad, DcMotor motor, ButtonControl buttonControl, ButtonControl buttonControl2, float power) {
        this(opMode,gamepad,motor,buttonControl,buttonControl2,power,true);
    }


    /**
     * Update motors with latest gamepad state
     */
    public void update() {
        // Only toggle when the button state changes from false to true, ie when the
        // button is pressed down (and not when the button comes back up)
        boolean pressed = buttonPressed(gamepad, buttonControl);
        boolean pressed2 = buttonPressed(gamepad,buttonControl2);

        if (pressed && lastButtonState != pressed) {
            if (motorPower > 1 || motorPower < 1){
                motor.setPower(1);
                motorPower = 1;
            }else if (motorPower > 0 || motorPower < 0){
                motor.setPower(0);
                motorPower = 0;

            }


        }
        lastButtonState = pressed;

        if (pressed2 && lastButtonState2 != pressed2){
            if (motor.getPower() > -1 || motor.getPower() < -1){
                motor.setPower(-1);
                motorPower = -1;
            }else if (motor.getPower() > 0 || motor.getPower() < 0){
                motor.setPower(0);
                motorPower = 0;
            }
        }
        lastButtonState2 = pressed2;
        getOpMode().telemetry.log().add("%s motor power: %.2f", pressed, motor.getPower(), motorPower);
        getOpMode().telemetry.log().add("%s motor power: %.2f", pressed2, motor.getPower(),motorPower);

    }


}

