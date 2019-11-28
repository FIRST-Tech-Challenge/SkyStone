package org.firstinspires.ftc.robotlib.debug;

import android.graphics.Bitmap;
import android.util.Base64;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.concurrent.ExecutorService;

public class DebugServer {
    private Webserver webserver;
    HardwareMap hardwareMap;
    private ExecutorService cameraSevice;

    private CameraStreamSource streamSource;
    private String streamJPEG64;

    public DebugServer(HardwareMap hardwareMap, CameraStreamSource streamSource) {
        this.cameraSevice = ThreadPool.newSingleThreadExecutor("camStreamService");
        this.hardwareMap = hardwareMap;
        this.streamSource = streamSource;
    }

    public void init() throws IOException {
        this.webserver = new Webserver(this);
        this.cameraSevice.submit(new CameraManager());
    }

    // Camera Stuff
    private class CameraManager implements Runnable {
        private String convertBitmapToJPEG64(Bitmap bitmap) {
            ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
            bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
            return Base64.encodeToString(outputStream.toByteArray(), Base64.DEFAULT);
        }

        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                streamSource.getFrameBitmap(Continuation.createTrivial(new Consumer<Bitmap>() {
                    @Override
                    public void accept(final Bitmap frame) {
                        setStreamJPEG64(convertBitmapToJPEG64(frame));
                    }
                }));
            }
        }
    }

    public void shutdownCameraThread() {
        this.cameraSevice.shutdown();
    }

    public String getStreamJPEG64() {
        return streamJPEG64;
    }

    private void setStreamJPEG64(String base64) {
        streamJPEG64 = base64;
    }
}
