package teamcode.impl;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import teamcode.common.TTHardwareManager;
import teamcode.common.TTOpMode;

@TeleOp(name = "TT Arm Distance Sensor Test")
public class DistanceSensorArmTest extends TTOpMode {

    @Override
    protected void onInitialize() {
        setHardwareRestriction(TTHardwareManager.TTHardwareRestriction.ARM_ONLY);
    }

    @Override
    protected void onStart() {
        DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "ArmLiftSensor");
        getRobot().getArm().setLiftHeight(11, 1.0);
        getRobot().getArm().setLiftHeight(0, 1.0);
//        while (opModeIsActive()) {
//            telemetry.addData("Distance Sensor Reading", sensor.getDistance(DistanceUnit.INCH));
//            telemetry.update();
//        }
    }

}
