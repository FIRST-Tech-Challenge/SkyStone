package org.firstinspires.ftc.teamcode.monitor;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.DeviceMap;
import org.firstinspires.ftc.teamcode.listener.CameraListener;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

public class MonitorCamera implements IMonitor {
    private final List<CameraListener> listeners = new ArrayList<>();
    private final ScheduledExecutorService executor = Executors.newScheduledThreadPool(2);
    private final VuforiaLocalizer vuforia;
    private Telemetry telemetry;
    private static Image currentImage;
    private boolean started = false;

    public MonitorCamera(DeviceMap map) {
        this.vuforia = map.getVuforia();
        this.telemetry = DeviceMap.getTelemetry();
        if(vuforia == null) throw new RuntimeException("Error!");
        this.started = false;


    }

    public void addListener(CameraListener listener) {
        listener.setup();
        listeners.add(listener);
        telemetry.addLine(listeners.size() + "LI SIZE");
    }

    @Override
    public void start() {
        if(started) return;//throw new RuntimeException("This method cannot be called twice! Camera:26");
        started = true;

        new Thread(() -> {
            while(started) {
                this.run();
            }
        }).start();
        //executor.run
        //this.executor.scheduleAtFixedRate(this, 0, 10, TimeUnit.MILLISECONDS);
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
        RobotLog.d("UltroTag", "run monitor");
        telemetry.addLine(Math.random() + "");
        telemetry.addLine("-10");
        VuforiaLocalizer.CloseableFrame frame = null;
        try {
            frame = vuforia.getFrameQueue().take();
            telemetry.addLine("-9");
            updateImage(frame);
        }catch (Exception e) {
            RobotLog.d(e.getMessage());
            e.printStackTrace();
        }finally {
            if(frame != null)
                frame.close();
        }
    }

    private void updateImage(VuforiaLocalizer.CloseableFrame frame) {
        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            telemetry.addLine("-8");
            Image image = frame.getImage(i);
            if (image.getFormat() == PIXEL_FORMAT.RGB565) {
                telemetry.addLine("-7");
                currentImage = image;
                convertMat(image);
                telemetry.addLine("-6");
                telemetry.update();
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
    private void convertMat(Image image)  {
        try {
            telemetry.addLine("0");
            telemetry.addLine("1");
            Bitmap bm = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(image.getPixels());

            telemetry.addLine("1");
            // construct an OpenCV mat from the bitmap using Utils.bitmapToMat()
            Mat mat = new Mat(bm.getWidth(), bm.getHeight(), CvType.CV_8UC4);
            Utils.bitmapToMat(bm, mat);

            telemetry.addData("listeners: ", listeners.size());
            telemetry.update();
            for (CameraListener cameraListener : listeners) {
                cameraListener.process(image, mat);
            }
            throw new Exception("aawfawfaw");
        }catch (Exception e) {
            e.printStackTrace();
        }
    }
}
