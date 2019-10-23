package teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "MotorTest")
public class TestMotorOpMode extends TTOpMode{
    private DcMotor motor1;
    @Override
    protected void onInitialize() {
        motor1 = hardwareMap.get(DcMotor.class, "ArmLift");
    }

    @Override
    protected void onStart() {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setTargetPosition(10000);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(motor1.isBusy()) {
            motor1.setPower(1);
            telemetry.addData("Tick Value: ", motor1.getCurrentPosition());
            telemetry.update();
        }
        motor1.setPower(0);

    }

    @Override
    protected void onStop() {

    }
}
