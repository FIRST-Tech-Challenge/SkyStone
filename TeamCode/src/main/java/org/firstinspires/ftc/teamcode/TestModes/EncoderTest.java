package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Disabled

//@TeleOp(name = "EncoderTest")


public class EncoderTest extends OpMode {

    DcMotor first;
    DigitalChannel button;
    boolean lastState = false;

    @Override
    public void init() {
        first = hardwareMap.dcMotor.get("motor");
        button = hardwareMap.digitalChannel.get("button");

        first.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        first.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        first.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        first.setPower(0);
    }

    @Override
    public void loop() {
        telemetry.addData("EncoderPosition:", first.getCurrentPosition());
        telemetry.addData("lastState:", lastState);
        telemetry.addData("Button:", button.getState());
        telemetry.update();


        if (button.getState() == false) {
            if (lastState) {
                first.setPower(0);
                lastState = false;
            } else {
                first.setPower(1);
                lastState = true;
            }
            while (!button.getState()) {}
        }
    }
}


