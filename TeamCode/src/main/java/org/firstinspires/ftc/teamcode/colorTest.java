package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "colorTest")
//@Disabled
public class colorTest extends LinearOpMode {

    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.get(ColorSensor.class, "cs");

        colorSensor.enableLed(true);
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Green Value: ", colorSensor.green());
            telemetry.addData("Blue Value: ", colorSensor.blue());
            telemetry.addData("Red Value: ", colorSensor.red());
            telemetry.addData("Hue: ", colorSensor.argb());
            telemetry.update();
        }
    }
}
