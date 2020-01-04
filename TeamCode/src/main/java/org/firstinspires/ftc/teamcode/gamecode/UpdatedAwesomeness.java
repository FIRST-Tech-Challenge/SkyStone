package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Robot;

/**
 * Created by Alec Krawciw on 2017-08-23.
 */

public class UpdatedAwesomeness extends AutoOpMode {
    @Override
    public void runOp() throws InterruptedException {
        Robot ceaser = new Robot();
        ceaser.reverseDriveSystem();

        ColorSensor sensor = RC.h.colorSensor.get("color");
        sensor.enableLed(false);

        waitForStart();

        while (sensor.red() < 100 && sensor.blue() < 100 && opModeIsActive()) {
            ceaser.forward(0.15);

        }

        if (sensor.red() > 100){
            ceaser.backward(0.5, 500);
            ceaser.imuTurnL(90, 0.5);
            ceaser.stop();
        }
        if (sensor.blue() > 100) {

            ceaser.backward(0.5, 500);
            ceaser.imuTurnR(90, 0.5);
            ceaser.stop();
        }

    }
}
