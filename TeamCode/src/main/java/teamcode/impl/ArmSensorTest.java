package teamcode.impl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.common.League1TTArm;
import teamcode.common.TTOpMode;

@TeleOp(name = "Arm Sensor Test")
public class ArmSensorTest extends TTOpMode {

    private League1TTArm arm;

    protected void onInitialize() {
        arm = new League1TTArm(hardwareMap);
    }

    @Override
    protected void onStart() {
        while (opModeIsActive()) {
            if (gamepad1.y) {
                arm.raise(0.5);
            } else if (gamepad1.a) {
                arm.lower(0.5);
            }
            arm.testColorSensor(telemetry);
        }
    }

    @Override
    protected void onStop() {

    }

}
