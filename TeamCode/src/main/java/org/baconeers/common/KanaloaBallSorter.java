package org.baconeers.common;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Operation to assist with Gamepad actions on DCMotors
 */
public class KanaloaBallSorter extends BaconComponent {


    private final ColorSensor colorsensor;
    private final Servo servo;
    private final Telemetry.Item item;
    private double red,blue;
    private boolean showTelemetry;



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
    public KanaloaBallSorter(BaconOpMode opMode, ColorSensor colorSensor, Servo servo, boolean showTelemetry) {
        super(opMode);

        this.colorsensor = colorSensor;
        this.servo = servo;
        this.showTelemetry = showTelemetry;


        if (showTelemetry) {
            item = opMode.telemetry.addData("red", 0.0f);
            item.setRetained(true);
        } else {
            item = null;
        }
    }
    public KanaloaBallSorter(BaconOpMode opMode, ColorSensor colorSensor, Servo servo) {
        this(opMode, colorSensor, servo,true);
    }

    public void init() {
        colorsensor.enableLed(true);
    }

    /**
     * Update motors with latest gamepad state
     */
    public void update() {
        // Only toggle when the button state changes from false to true, ie when the
        // button is pressed down (and not when the button comes back up)
        red = colorsensor.red();
        blue = colorsensor.blue();

        if (red > 150)
        {
            servo.setPosition(0.95);

        }else if(blue > 150){
            servo.setPosition(0.85);
        }
            if (item != null) {
                item.setValue(red);
            }
            if (showTelemetry) {
                getOpMode().telemetry.log().add("%s red value: %.1f blue value: %.1f %.3f", "color", red, blue, getOpMode().time);
            }
    }


}
