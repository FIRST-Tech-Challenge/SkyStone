package org.firstinspires.ftc.teamcode.PID.localizer;

import android.os.SystemClock;
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
    private String TAG = "IMUBufferReader";
    private float lastGyroValue = 0;
    private long lastGyroTime = 0;
    private static final int IMU_FRESH_THRESHOLD = 10; // milli seconds
    private static boolean keepRunning = true;
    private boolean IMUReaderRunning = false;
    private float[] pingPongBuffer = new float[2];
    private int latestIndex = 0;
    private float gyro_range = 2 * (float) Math.PI;

    Thread thread;

    private IMUBufferReader(HardwareMap hardwareMap) {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        resetIMU();
        while(!imu.isGyroCalibrated())
        {
            RobotLogger.dd(TAG, "IMU calibrating");
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        RobotLogger.dd(TAG, "IMU calibrating done");

        thread = new Thread(this);
        thread.start();
        IMUReaderRunning = true;
    }

    synchronized  public static IMUBufferReader getSingle_instance(HardwareMap hardwareMap)
    {
        if (single_instance == null) {
            single_instance = new IMUBufferReader(hardwareMap);
        }

        return single_instance;
    }

    synchronized  public static void cleanUP()    {
        stop_running();
        single_instance = null;
    }

    public void finalize() throws Throwable{
        IMUBufferReader.stop_running();
    }

    private static void stop_running()
    {
        keepRunning = false;
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    private boolean resetIMU()
    {
        return(imu.initialize(parameters));
    }

    synchronized public float getLatestIMUData()
    {
        float v;
        if (false)//imu.getSystemError() != BNO055IMU.SystemError.NO_ERROR)
        {
            RobotLogger.dd(TAG, "IMU error");
            v = lastGyroValue;
        }
        else
        {
            long delta = SystemClock.elapsedRealtime() - lastGyroTime;
            RobotLogger.dd(TAG, "IMU gyro time delta: " + delta);
            if (delta > IMU_FRESH_THRESHOLD) {
                v = imu.getAngularOrientation().firstAngle;

                v = ((v % gyro_range) + gyro_range) % gyro_range;

                lastGyroTime = SystemClock.elapsedRealtime();
                lastGyroValue = v;
            }
            else
                v = lastGyroValue;
        }

        /*
        if (IMUReaderRunning == false)
        {
            RobotLogger.dd(TAG, "IMU reader started");
            thread = new Thread(this);
            thread.start();
            IMUReaderRunning = true;
        }
        float t = pingPongBuffer[0];
        try {
            mutex.acquire();
            t = pingPongBuffer[latestIndex];
            mutex.release();
        }
        catch (InterruptedException exc) {
            System.out.println(exc);
        }
        */

        return v;
    }

    public void run(){
        // polling in dedicated thread is not good idea;
        IMUReaderRunning = true;
        return;
        /*
        while (IMUBufferReader.keepRunning) {
            try {
                if (imu.getSystemError() != BNO055IMU.SystemError.NO_ERROR)
                {
                    keepRunning = false;
                    RobotLogger.dd(TAG, "IMU error, stop thread");
                    continue;
                }
                float t = imu.getAngularOrientation().firstAngle;

                mutex.acquire();

                if (latestIndex == 0)
                    latestIndex = 1;
                else
                    latestIndex = 0;
                pingPongBuffer[latestIndex] = t;
                mutex.release();

                Thread.sleep((long) DriveConstantsPID.imuPollingInterval);
            } catch (Throwable e) {
                IMUReaderRunning = false;
                keepRunning = false;
                RobotLogger.dd(TAG, "IMU read failure");
                System.out.println(e);
            }
        }
        RobotLogger.dd(TAG, "IMU thread stops");
         */
    }
}
