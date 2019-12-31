package org.firstinspires.ftc.teamcode.Experimental.AccelerometerDeadReckoning;

import android.hardware.Sensor;
import android.hardware.SensorManager;

import com.qualcomm.ftccommon.FtcRobotControllerService;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

@Disabled
@TeleOp(name = "phone accelerometer test")

// Both this and built-in rev accelerometer are completely useless for dead reckoning, maybe revisit after coming up with a better integration algorithm
public class PhoneAccelerometerTest extends LinearOpMode {

    PhoneAccelerometerLocalizer localizer = new PhoneAccelerometerLocalizer();

    @Override
    public void runOpMode() throws InterruptedException {
        SensorManager sensors = FtcRobotControllerActivity.sensors;
        if (sensors.getSensorList(Sensor.TYPE_LINEAR_ACCELERATION).size() != 0) {
            Sensor accelerometer = sensors.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
            sensors.registerListener(localizer, accelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        }
        else {
            requestOpModeStop();
            telemetry.addData("Accelerometer", "Could not bind accelerometer");
            telemetry.update();
        }

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Accelerometer", localizer.motionState.toString());
            telemetry.update();
        }

        sensors.unregisterListener(localizer);
    }
}
