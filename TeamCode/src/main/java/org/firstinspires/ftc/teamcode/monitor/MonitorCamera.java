package org.firstinspires.ftc.teamcode.monitor;

import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.DeviceMap;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class MonitorCamera implements IMonitor {
    private final ScheduledExecutorService executor;
    private final VuforiaLocalizer vuforia;

    private static Image currentImage;
    private boolean started = false;

    public MonitorCamera(DeviceMap map) {
        this.vuforia = map.getVuforia();
        this.executor = Executors.newScheduledThreadPool(4);
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

    @Override
    public void run() {
        VuforiaLocalizer.CloseableFrame frame = null;
        try {
            frame = vuforia.getFrameQueue().take();
            if(frame != null)
                updateImage(frame);
        }catch (InterruptedException|NullPointerException e) {
            RobotLog.d(e.getMessage());
            e.printStackTrace();
        }finally {
            if(frame != null)
                frame.close();
        }
    }

    private static void updateImage(VuforiaLocalizer.CloseableFrame frame) {
        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            Image image = frame.getImage(i);
            if(image == null) continue;
            if (image.getFormat() == PIXEL_FORMAT.RGB565) {
                currentImage = image;
                frame.close();
                return;
            }
        }
        currentImage = null;
    }
}
