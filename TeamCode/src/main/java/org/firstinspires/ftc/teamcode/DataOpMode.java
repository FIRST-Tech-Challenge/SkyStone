package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;

@Autonomous(name = "DATA VALUES", group = "")
public class DataOpMode extends AutoOpMode  {
    @Override
    public void setup(DeviceMap mapper) {
        mapper.setUpMotors(hardwareMap);
        mapper.servoInit(hardwareMap);
        mapper.sensorInit(hardwareMap);


    }

    @Override
    public void beforeLoop() {
        DeviceMap map = DeviceMap.getInstance();

        for(DcMotor motor : map.getAllMotors()) {
            telemetry.addData("motor: ", motor.getCurrentPosition());
        }
        for(Servo servo : map.getServos()) {
            telemetry.addData("servo:", servo.getPosition());
        }
        for(ColorSensor colorSensor : map.getColorSensors()) {
            telemetry.addData("colorsensor (r, g, b): ", String.format("%d, %d, %d", colorSensor.red(), colorSensor.green(), colorSensor.blue()));
        }
        for(DistanceSensor distanceSensor : map.getDistanceSensors()) {
            telemetry.addData("Distance sensor ", distanceSensor.getDistance(DistanceUnit.CM));
        }
        telemetry.update();
    }

    @Override
    public void run() {

    }
}
