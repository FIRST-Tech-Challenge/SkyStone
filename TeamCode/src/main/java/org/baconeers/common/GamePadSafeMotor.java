package org.baconeers.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Operation to assist with Gamepad actions on DCMotors
 */
public class GamePadSafeMotor extends BaconComponent {

    private final ButtonControl buttonControl1;
    private final ButtonControl buttonControl2;
    private final DcMotor motor;
    private final Gamepad gamepad;
    private final float motorPower;
    private boolean motorOn = false;
    private boolean lastButtonState = false;
    private final Telemetry.Item item;


    /**
     * Constructor for operation.  Telemetry enabled by default.
     *
     * @param opMode
     * @param gamepad        Gamepad
     * @param motor          DcMotor to operate on
     * @param buttonControl1 {@link ButtonControl}
     * @param power          power to apply when using gamepad buttons
     * @param showTelemetry  display the power values on the telemetry
     */
    public GamePadSafeMotor(BaconOpMode opMode, Gamepad gamepad, DcMotor motor,
                            ButtonControl buttonControl1, ButtonControl buttonControl2, float power,
                            boolean showTelemetry) {
        super(opMode);

        this.gamepad = gamepad;
        this.motor = motor;
        this.buttonControl1 = buttonControl1;
        this.buttonControl2 = buttonControl2;
        this.motorPower = power;

        if (showTelemetry) {
            item = opMode.telemetry.addData("Control " + buttonControl1.name() + " + " + buttonControl2.name(), 0.0f);
            item.setRetained(true);
        } else {
            item = null;
        }
    }

    public GamePadSafeMotor(BaconOpMode opMode, Gamepad gamepad, DcMotor motor,
                            ButtonControl buttonControl1, ButtonControl buttonControl2, float power) {
        this(opMode,gamepad,motor,buttonControl1,buttonControl2,power,true);
    }


    /**
     * Update motors with latest gamepad state
     */
    public void update() {
        // Only engage the motor when both buttons are pressed as a form of
        // safety to prevent accidental triggering of the motor
        motorOn = buttonPressed(gamepad, buttonControl1) && buttonPressed(gamepad, buttonControl2);
        if (motorOn != lastButtonState) {
            float power = motorOn ? motorPower : 0.0f;
            motor.setPower(power);
            if (item != null) {
                item.setValue(power);
            }
            lastButtonState = motorOn;

            getOpMode().telemetry.log().add("%s+%s motor power: %.2f", buttonControl1.name(), buttonControl2.name(), power);
        }
    }

}
