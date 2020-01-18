package org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.DutchFTCCore.Robot;
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
                //TODO: Implement this instead of the seperate opmode for calibrating
    }

    @Override
    public void Update() {
        super.Update();
        //polls the sensor directly, converts from byte array to double
        byte[] a = Robot.instance.imu.read(BNO055IMU.Register.EUL_H_LSB, 2);
        currHeading = ByteBuffer.wrap(a).getDouble();

    }
}
