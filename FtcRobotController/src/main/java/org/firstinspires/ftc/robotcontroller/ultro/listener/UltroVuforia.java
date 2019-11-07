package org.firstinspires.ftc.robotcontroller.ultro.listener;

import android.graphics.Bitmap;

import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.State;
import com.vuforia.Trackable;
import com.vuforia.TrackableResult;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.function.ContinuationResult;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaTrackableContainer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaTrackableImpl;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaTrackableNotify;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaTrackablesImpl;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class UltroVuforia extends VuforiaLocalizerImpl {
    boolean closed = false;
    private static final List<CameraListener> listeners = new ArrayList<>();

    public static void addListener(CameraListener listener) {
        listeners.clear();
        listener.setup();
        listeners.add(listener);
    }

    @Override
    public void close() {
        if (!closed) super.close();
        closed = true;
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
            synchronized (updateCallbackLock) {
                callbackCount++;

                // If the user wants to see frames, then give him the new one. Convert it to
                // a CloseableFrame so as to (a) make it liveable beyond the callback lifetime, and
                // (b) expose a close() method that can be used to proactively reclaim memory.
                Continuation<? extends Consumer<Frame>> capturedContinuation = null;
                synchronized (frameQueueLock) {
                    if (frameQueueCapacity > 0) {
                        CloseableFrame closeableFrame = new CloseableFrame(state.getFrame());
                        frameQueue.add(closeableFrame);
                    }

                    capturedContinuation = getFrameOnce;
                    getFrameOnce = null;
                }
                if (capturedContinuation != null) {
                    final CloseableFrame closeableFrame = new CloseableFrame(state.getFrame());
                    updateImage(closeableFrame);
                    capturedContinuation.dispatch(new ContinuationResult<Consumer<Frame>>()
                    {
                        @Override public void handle(Consumer<Frame> frameConsumer)
                        {
                            frameConsumer.accept(closeableFrame);
                            closeableFrame.close();
                        }
                    });
                }

                // Figure out which of our trackables are visible and which are not. Let each
                // one know its status.

                Set<VuforiaTrackable> notVisible = new HashSet<>();

                synchronized (loadedTrackableSets) {
                    for (VuforiaTrackablesImpl trackables : loadedTrackableSets) {
                        for (VuforiaTrackable vuforiaTrackable : trackables) {
                            // Add the trackable itself
                            notVisible.add(vuforiaTrackable);

                            // Robustly take care of any parent / child relationships.
                            if (vuforiaTrackable instanceof VuforiaTrackableContainer)
                            {
                                notVisible.addAll(((VuforiaTrackableContainer)vuforiaTrackable).children());
                            }
                            VuforiaTrackable parent = vuforiaTrackable.getParent();
                            if (parent != null)
                            {
                                notVisible.add(parent);
                            }
                        }
                    }
                }

                int numTrackables = state.getNumTrackableResults();
                for (int i = 0; i < numTrackables; i++) {
                    TrackableResult trackableResult = state.getTrackableResult(i);
                    if (isObjectTargetTrackableResult(trackableResult)) {
                        Trackable trackablea = trackableResult.getTrackable();
                        if (trackablea != null) {
                            VuforiaTrackable vuforiaTrackable = VuforiaTrackableImpl.from(trackableResult);
                            if (vuforiaTrackable != null) {
                                notVisible.remove(vuforiaTrackable);
                                VuforiaTrackable parent = vuforiaTrackable.getParent();
                                if (parent != null) {
                                    notVisible.remove(parent);
                                }
                                if (vuforiaTrackable instanceof VuforiaTrackableNotify) {
                                    ((VuforiaTrackableNotify)vuforiaTrackable).noteTracked(trackableResult, getCameraName(), getCamera());
                                }
                            } else
                                tracer.trace("vuforiaTrackable unexpectedly null: %s", trackableResult.getClass().getSimpleName());
                        } else
                            tracer.trace("trackable unexpectedly null: %s", trackableResult.getClass().getSimpleName());
                    } else
                        tracer.trace("unexpected TrackableResult: %s", trackableResult.getClass().getSimpleName());
                }

                for (VuforiaTrackable vuforiaTrackable : notVisible) {
                    if (vuforiaTrackable instanceof VuforiaTrackableNotify) {
                        ((VuforiaTrackableNotify)vuforiaTrackable).noteNotTracked();
                    }
                }
            }
        }
    }


    private void updateImage(VuforiaLocalizer.CloseableFrame frame) {
        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            Image image = frame.getImage(i);
            if (image.getFormat() == PIXEL_FORMAT.RGB565) {
                try {
                    //convertMat(image);
                }catch (Exception e) {
                    e.printStackTrace();
                }
                break;
            }
        }
    }

/*
    private void convertMat(Image image)  {
        Bitmap bm = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);

        bm.copyPixelsFromBuffer(image.getPixels());


        // construct an OpenCV mat from the bitmap using Utils.bitmapToMat()

        Mat mat = new Mat(bm.getWidth(), bm.getHeight(), CvType.CV_8UC4);

        Utils.bitmapToMat(bm, mat);

        for(CameraListener listener : listeners) listener.process(image, mat);
    }

 */
}
