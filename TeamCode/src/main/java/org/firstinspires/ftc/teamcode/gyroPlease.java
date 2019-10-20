package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.I2cAddr;


@TeleOp(name = "gyroPlease", group = "gryoPlease")
public class gyroPlease extends OpMode {
    ElapsedTime runtime = new ElapsedTime();

    ModernRoboticsI2cGyro cartGyro;
    private I2cAddr i2cAddrGyro = I2cAddr.create8bit(0x20);




    @Override
    public void init() {
        cartGyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("cartGyro");
        cartGyro.setI2cAddress(i2cAddrGyro);
        cartGyro.calibrate();

        runtime.reset();
/*        do {
            gyro.calibrate();
        }
        while (gyro.isCalibrating());*/
    }

    @Override
    public void loop() {

        telemetry.addData("Gyro Stat", cartGyro.status());
        telemetry.addData("Gyro Cal", cartGyro.isCalibrating());
        telemetry.addData("Heading", cartGyro.getHeading());
        telemetry.addData("rawX", cartGyro.rawX());
        telemetry.addData("rawY", cartGyro.rawY());
        telemetry.addData("rawZ", cartGyro.rawZ());
        telemetry.update();

    }

    @Override
    public void stop() {
        super.stop();
    }
}

