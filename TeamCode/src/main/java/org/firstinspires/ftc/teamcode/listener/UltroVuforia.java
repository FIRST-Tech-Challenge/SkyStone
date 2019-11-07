package org.firstinspires.ftc.teamcode.listener;

import android.graphics.Bitmap;
import android.net.sip.SipSession;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.State;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

public class UltroVuforia extends VuforiaLocalizerImpl {
    private static final List<CameraListener> listeners = new ArrayList<>();

    public static void addListener(CameraListener listener) {
        listener.setup();
        listeners.add(listener);
    }
    public UltroVuforia(Parameters parameters) {
        super(parameters);
        stopAR();

        this.vuforiaCallback = new UltroCallback();
        startAR();

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
    }

    public class UltroCallback extends VuforiaLocalizerImpl.VuforiaCallback {
        @Override
        public void Vuforia_onUpdate(State state) {
            super.Vuforia_onUpdate(state);


            try {
                CloseableFrame frame = getFrameQueue().take();
                updateImage(frame);
                frame.close();
            }catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }


    private void updateImage(VuforiaLocalizer.CloseableFrame frame) {
        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            Image image = frame.getImage(i);
            if (image.getFormat() == PIXEL_FORMAT.RGB565) {
                try {
                    convertMat(image);
                }catch (Exception e) {
                    e.printStackTrace();
                }
                break;
            }
        }
    }


    private void convertMat(Image image)  {
        Bitmap bm = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);

        bm.copyPixelsFromBuffer(image.getPixels());


        // construct an OpenCV mat from the bitmap using Utils.bitmapToMat()

        Mat mat = new Mat(bm.getWidth(), bm.getHeight(), CvType.CV_8UC4);

        Utils.bitmapToMat(bm, mat);

        for(CameraListener listener : listeners) listener.process(image, mat);
    }
}
