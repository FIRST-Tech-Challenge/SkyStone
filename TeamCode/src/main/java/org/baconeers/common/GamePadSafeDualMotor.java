package org.baconeers.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Operation to assist with Gamepad actions on DCMotors
 */
public class GamePadSafeDualMotor extends BaconComponent {

    private final ButtonControl buttonControl1;
    private final ButtonControl buttonControl2;
    private final DcMotor motor1;
    private final DcMotor motor2;
    private final Gamepad gamepad;
    private final float motorPower;
    private boolean motorOn = false;
    private boolean lastButtonState = false;
    private final Telemetry.Item item;
    private boolean showTelemetry = true;



    /**
     * Constructor for operation.  Telemetry enabled by default.
     *
     * @param opMode
     * @param gamepad        Gamepad
     * @param motor1          First DcMotor to operate on
     * @param motor2          Second DcMotor to operate on
     * @param buttonControl1 {@link ButtonControl}
     * @param power          power to apply when using gamepad buttons
     * @param showTelemetry  display the power values on the telemetry
     */
    public GamePadSafeDualMotor(BaconOpMode opMode, Gamepad gamepad, DcMotor motor1, DcMotor motor2,
                                ButtonControl buttonControl1, ButtonControl buttonControl2, float power,
                                boolean showTelemetry) {
        super(opMode);

        this.gamepad = gamepad;
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.buttonControl1 = buttonControl1;
        this.buttonControl2 = buttonControl2;
        this.motorPower = power;
        this.showTelemetry = showTelemetry;

        if (showTelemetry) {
            item = opMode.telemetry.addData("Control " + buttonControl1.name() + " + " + buttonControl2.name(), 0.0f);
            item.setRetained(true);
        } else {
            item = null;
        }
    }

    public GamePadSafeDualMotor(BaconOpMode opMode, Gamepad gamepad, DcMotor motor1, DcMotor motor2,
                                ButtonControl buttonControl1, ButtonControl buttonControl2, float power) {
        this(opMode,gamepad, motor1, motor2, buttonControl1,buttonControl2,power,true);
    }


    /**
     * Update motors with latest gamepad state
     */
    public void update() {
        // Only engage the motor1 when both buttons are pressed as a form of
        // safety to prevent accidental triggering of the motor1
        motorOn = buttonPressed(gamepad, buttonControl1) && buttonPressed(gamepad, buttonControl2);
        if (motorOn != lastButtonState) {
            float power = motorOn ? motorPower : 0.0f;
            motor1.setPower(power);
            motor2.setPower(power);
            if (item != null) {
                item.setValue(power);
            }
            lastButtonState = motorOn;

            if (showTelemetry) {
                getOpMode().telemetry.log().add("%s+%s dual motor power: %.2f", buttonControl1.name(), buttonControl2.name(), power);
            }
        }
    }

}
