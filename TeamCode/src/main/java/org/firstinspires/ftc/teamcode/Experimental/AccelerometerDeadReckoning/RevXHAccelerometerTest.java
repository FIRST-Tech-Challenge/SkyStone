package org.firstinspires.ftc.teamcode.Experimental.AccelerometerDeadReckoning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.All.HardwareMap;

@Disabled
@TeleOp(name = "RevXH Accelerometer Test")

// Both this and phone accelerometer are completely useless for dead reckoning, maybe revisit after coming up with a better integration algorithm
public class RevXHAccelerometerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareMap hw = new HardwareMap(hardwareMap);
        hw.gyroInit();
        RevXHAccelerometerLocalizer localizer = new RevXHAccelerometerLocalizer(hw.gyro);
        waitForStart();
        while (opModeIsActive()) {
            localizer.update();
            telemetry.addData("Rev Accelerometer Data", localizer.movementState.toString());
            telemetry.update();
        }
    }
}
