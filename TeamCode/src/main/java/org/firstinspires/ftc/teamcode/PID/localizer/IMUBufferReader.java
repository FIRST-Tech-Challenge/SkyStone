package org.firstinspires.ftc.teamcode.PID.localizer;

import android.support.annotation.NonNull;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.util.LynxModuleUtil;

import java.util.concurrent.Semaphore;

public class IMUBufferReader implements Runnable{
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private static IMUBufferReader single_instance = null;

    private Semaphore mutex = new Semaphore(1);

    private boolean IMUReaderRunning = false;
    private float[] pingPongBuffer = new float[2];
    private int latestIndex = 0;
    Thread thread;

    public IMUBufferReader(HardwareMap hardwareMap) {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        resetIMU();

        IMUReaderRunning = true;
        thread = new Thread(this);
        thread.start();
    }
    public boolean resetIMU()
    {
        return(imu.initialize(parameters));
    }
    public static IMUBufferReader getSingle_instance(HardwareMap hardwareMap)
    {
        if (single_instance == null) {
            single_instance = new IMUBufferReader(hardwareMap);
        }

        return single_instance;
    }
    public float getLatestIMUData()
    {
        float t = pingPongBuffer[0];
        try {
            mutex.acquire();
            t = pingPongBuffer[latestIndex];
            mutex.release();
        }
        catch (InterruptedException exc) {
            System.out.println(exc);
        }

        return t;
    }

    public void run(){
        try {
            float t = imu.getAngularOrientation().firstAngle;
            mutex.acquire();

            if (latestIndex == 0)
                latestIndex = 1;
            else
                latestIndex = 0;
            pingPongBuffer[latestIndex] = t;
            mutex.release();
            Thread.sleep((long)DriveConstantsPID.imuPollingInterval);
        }
        catch (InterruptedException exc) {
            System.out.println(exc);
        }
    }
}
