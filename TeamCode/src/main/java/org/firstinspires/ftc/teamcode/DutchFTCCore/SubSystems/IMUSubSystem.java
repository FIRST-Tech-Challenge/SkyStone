package org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.DutchFTCCore.Robot;

import java.io.File;
import java.nio.ByteBuffer;

public class IMUSubSystem extends SubSystem {

    public static IMUSubSystem instance;
    public static double currHeading;

    @Override
    public void Start() {
        instance = this;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.pitchMode = BNO055IMU.PitchMode.ANDROID;
        Robot.instance.imu.initialize(parameters);
    }

    public void Calibrate () {
        BNO055IMU.CalibrationData calibrationData = Robot.instance.imu.readCalibrationData();

        // Save the calibration data to a file. You can choose whatever file
        // name you wish here, but you'll want to indicate the same file name
        // when you initialize the IMU in an opmode in which it is used. If you
        // have more than one IMU on your robot, you'll of course want to use
        // different configuration file names for each.
        String filename = "AdafruitIMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());
    }

    @Override
    public void Update() {
        super.Update();
        //polls the sensor directly, converts from byte array to double
        byte[] a = Robot.instance.imu.read(BNO055IMU.Register.EUL_H_LSB, 2);
        currHeading = ByteBuffer.wrap(a).getDouble();

    }
}
