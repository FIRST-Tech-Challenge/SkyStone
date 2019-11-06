package org.firstinspires.ftc.teamcode.monitor;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.DeviceMap;
import org.firstinspires.ftc.teamcode.listener.CameraListener;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class MonitorCamera implements IMonitor {
    private static final List<CameraListener> listeners = new ArrayList<>();
    private final ScheduledExecutorService executor;
    private final VuforiaLocalizer vuforia;

    private static Image currentImage;
    private boolean started = false;

    public MonitorCamera(DeviceMap map) {
        this.vuforia = map.getVuforia();
        this.executor = Executors.newScheduledThreadPool(4);
    }

    public void addListener(CameraListener listener) {
        listener.setup();
        listeners.add(listener);
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
            RobotLog.d("ULTRO", e.getMessage());
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
                convertMat(image);
                frame.close();
                return;
            }
        }
        currentImage = null;
    }

    /**
     * Thanks https://gist.github.com/zylom/7d7efa6ff44e9e69e12d8309476627f7
     * @param image
     */
    private static void convertMat(Image image) {
        Bitmap bm = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(image.getPixels());

        // construct an OpenCV mat from the bitmap using Utils.bitmapToMat()
        Mat mat = new Mat(bm.getWidth(), bm.getHeight(), CvType.CV_8UC4);
        Utils.bitmapToMat(bm, mat);

        for(CameraListener cameraListener : listeners) {
            cameraListener.process(image, mat);
        }
    }
}
