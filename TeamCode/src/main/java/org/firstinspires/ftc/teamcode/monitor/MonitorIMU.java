package org.firstinspires.ftc.teamcode.monitor;

import android.app.ActivityManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DeviceMap;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

public class MonitorIMU implements IMonitor {
    private final ScheduledExecutorService executor;
    private final BNO055IMU imu;
    //roll, pitch, header or somethhing like that
    private static float[] xyz = new float[] {0, 0, 0};
    private boolean started;

    public MonitorIMU(BNO055IMU imu) {
        this.imu = imu;
        this.executor = Executors.newSingleThreadScheduledExecutor();

        this.started = false;
        this.start();
    }

    @Override
    public void start() {
        if(started) throw new RuntimeException("This method cannot be called twice! MonitorIMU:26");
        started = true;

        this.executor.schedule(this, 25, TimeUnit.MILLISECONDS);
    }

    @Override
    public void stop() {
        try {
            this.executor.awaitTermination(30, TimeUnit.SECONDS);
        }catch (InterruptedException e) {
            RobotLog.d(e.getMessage());
            e.printStackTrace();
        }
    }

    //x
    public static float getAngleFirst() {
        return xyz[0]; //third
    }
    //y
    public static float getAngleSecond() {
        return xyz[1]; //second
    }
    //z
    public static float getAngleThird() {
        return xyz[2]; //first
    }

    public static float[] getXYZAngle() {
        return xyz;
    }

    private void update() {

        Orientation orient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        xyz = new float[] {
                orient.firstAngle,
                orient.secondAngle,
                orient.thirdAngle
        };
    }

    @Override
    public void run() {
        update();
    }
}
