package teamcode.impl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.common.League1TTArm;
import teamcode.common.TTOpMode;

@TeleOp(name = "Distance Sensor Test")
public class DistanceSensorTest extends TTOpMode {

    private League1TTArm arm;

    protected void onInitialize() {
        arm = new League1TTArm(hardwareMap);
    }

    @Override
    protected void onStart() {
        while (opModeIsActive()) {
            if (gamepad1.y) {
                arm.liftContinuous(0.25);
            } else if (gamepad1.a) {
                arm.liftContinuous(-0.25);
            } else {
                arm.liftContinuous(0.0);
            }
            telemetry.addData("Lift height", arm.getLiftHeight());
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {

    }

}
