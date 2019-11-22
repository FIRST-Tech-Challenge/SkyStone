package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="Servo Tester")
@Disabled
public class TouchSensorTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        DigitalChannel digChannel = (DigitalChannel) hardwareMap.get("elevatorSwitch");
        while(!isStopRequested()){
            telemetry.addData("Switch: ", digChannel.getState());
            telemetry.update();
        }
    }
}
