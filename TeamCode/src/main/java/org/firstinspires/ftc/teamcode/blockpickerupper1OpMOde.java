package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "blockpickerupper1OpMOde", group = "")
public class blockpickerupper1OpMOde extends LinearOpMode {

    private ColorSensor sensorOTJ;
    private ColorSensor yellowBlockColorSensor;
    private Servo GripServo;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        float hue;
        int colorHSV;
        float sat;
        float val;

        sensorOTJ = hardwareMap.colorSensor.get("sensorOTJ");
        yellowBlockColorSensor = hardwareMap.colorSensor.get("yellowBlockColorSensor");
        GripServo = hardwareMap.servo.get("GripServo");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Display distance info.
                telemetry.addData("Dist to tgt (cm)", ((DistanceSensor) sensorOTJ).getDistance(DistanceUnit.CM));
                // Display reflected light.
                telemetry.addData("Red channel value", yellowBlockColorSensor.red());
                telemetry.addData("Blue channel value", yellowBlockColorSensor.blue());
                // Convert RGB values to HSV color model.
                // See https://en.wikipedia.org/wiki/HSL_and_HSV for details on HSV color model.
                colorHSV = Color.argb(yellowBlockColorSensor.alpha(), yellowBlockColorSensor.red(), yellowBlockColorSensor.green(), yellowBlockColorSensor.blue());
                // Get hue.
                hue = JavaUtil.colorToHue(colorHSV);
                telemetry.addData("Hue", hue);
                // Get saturation.
                sat = JavaUtil.colorToSaturation(colorHSV);
                telemetry.addData("Saturation", sat);
                // Get value.
                val = JavaUtil.colorToValue(colorHSV);
                telemetry.addData("Value", val);
                // Use hue to determine if it's red, green, blue, etc..
                if (hue < 60) {
                    GripServo.setPosition(0.25);
                }
                if (sat < 0.2) {
                    telemetry.addData("Check Sat", "Is surface white?");
                }
                telemetry.update();
                if (val < 0.16) {
                    telemetry.addData("Check Val", "Is surface black?");
                }
            }
            // Put run blocks here.
        }
    }
}