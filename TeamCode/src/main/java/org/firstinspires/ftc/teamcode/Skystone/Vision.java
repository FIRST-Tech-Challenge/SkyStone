package org.firstinspires.ftc.teamcode.Skystone;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.ConditionVariable;
import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;

import java.util.Locale;
import java.util.concurrent.ExecutorService;

public class Vision {
    private final String VUFORIA_KEY = "AbSCRq//////AAAAGYEdTZut2U7TuZCfZGlOu7ZgOzsOlUVdiuQjgLBC9B3dNvrPE1x/REDktOALxt5jBEJJBAX4gM9ofcwMjCzaJKoZQBBlXXxrOscekzvrWkhqs/g+AtWJLkpCOOWKDLSixgH0bF7HByYv4h3fXECqRNGUUCHELf4Uoqea6tCtiGJvee+5K+5yqNfGduJBHcA1juE3kxGMdkqkbfSjfrNgWuolkjXR5z39tRChoOUN24HethAX8LiECiLhlKrJeC4BpdRCRazgJXGLvvI74Tmih9nhCz6zyVurHAHttlrXV17nYLyt6qQB1LtVEuSCkpfLJS8lZWS9ztfC1UEfrQ8m5zA6cYGQXjDMeRumdq9ugMkS";

    public enum Location {
        CENTER, LEFT, RIGHT, UNKNOWN
    }

    File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    int captureCounter = 0;

    private VuforiaLocalizer vuforia;

    private LinearOpMode linearOpMode;

    public Vision(LinearOpMode linearOpmode) {
        this.linearOpMode = linearOpmode;
        initVision();
    }

    public void initVision() {
        initVuforia();
    }

    public static double calcAverageYellow(Bitmap bitmap, int x, int y, int width, int height) {
        double sum = 0;
        int endX = x + width;
        int endY = y + height;
        int intColor;
        int redGreen;

        for (int i = x; i < endX; i++) {
            for (int j = y; j < endY; j++) {
                intColor = bitmap.getPixel(i, j);
                redGreen = Color.red(intColor) + Color.green(intColor);
                sum += redGreen;
            }
        }

        return sum / (width * height);
    }

    public void captureFrameToFile(Bitmap bitmap) {
        if (bitmap != null) {
            File file = new File(captureDirectory, String.format(Locale.getDefault(), "VuforiaFrame-%d.png", captureCounter++));
            try {
                FileOutputStream outputStream = new FileOutputStream(file);
                try {
                    bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                } finally {
                    outputStream.close();
                    Log.d("Vision", "captured %s" + file.getAbsolutePath());
                }
            } catch (IOException e) {
                RobotLog.ee("TAG", e, "exception in captureFrameToFile()");
            }
        }
    }

    public void updateBitmapWithBoundingBoxes(Bitmap bitmap, int x, int y, int width, int height) {
        int endX = x + width;
        int endY = y + height;

        for (int i = x; i < endX; i++) {
            for (int j = y; j < endY; j++) {
                if (i == x || i == endX - 1) {
                    bitmap.setPixel(i, j, Color.argb(1, 0, 0, 0));
                } else if (j == y || j == endY - 1) {
                    bitmap.setPixel(i, j, Color.argb(1, 0, 0, 0));
                }
            }
        }
    }

    public Location runDetection(final boolean deactiviated, final boolean isRed) {
        if (vuforia != null) {
            final ArrayList<Location> resultLocation = new ArrayList<>();

            final long startTime = SystemClock.elapsedRealtime();

            final ExecutorService executorService = ThreadPool.getDefault();

            while (linearOpMode.opModeIsActive() && resultLocation.size() == 0 && SystemClock.elapsedRealtime() - startTime <= 250) {
                final ConditionVariable resultAvaliable = new ConditionVariable(false);

                vuforia.getFrameOnce(Continuation.create(executorService, new Consumer<Frame>() {
                    @Override
                    public void accept(Frame frame) {
                        //x = width 1280
                        //y = height 720
                        LinkedList<BoxDetection> detections = new LinkedList<>();

                        linearOpMode.telemetry.addLine(frame.toString());
                        linearOpMode.telemetry.update();

                        Bitmap bitmap = vuforia.convertFrameToBitmap(frame);

                        BoxDetection left = new BoxDetection(Location.LEFT);
                        BoxDetection center = new BoxDetection(Location.CENTER);
                        BoxDetection right = new BoxDetection(Location.RIGHT);
                        int width = 217;
                        int height = 90;
                        int gap = 50;
                        int startX;
                        int startY;

                        if (!isRed) {
                            startX = 210;
                            startY = 325;
                        } else {
                            startX = 344;
                            startY = 320;
                        }

                        if (bitmap != null) {
                            left.averageRedGreen = calcAverageYellow(bitmap, startX, startY, width, height);
                            center.averageRedGreen = calcAverageYellow(bitmap, startX + width + gap, startY, width, height);
                            right.averageRedGreen = calcAverageYellow(bitmap, startX + width * 2 + gap * 2, startY, width, height);

                            Log.d("Vision", "Left " + left.averageRedGreen + " Center: " + center.averageRedGreen + " Right: " + right.averageRedGreen);
                            linearOpMode.telemetry.addLine("Left " + left.averageRedGreen + " Center: " + center.averageRedGreen + " Right: " + right.averageRedGreen);

                            detections.add(left);
                            detections.add(center);
                            detections.add(right);

                            Collections.sort(detections, new Comparator<BoxDetection>() {
                                @Override
                                public int compare(BoxDetection boxDetection, BoxDetection t1) {
                                    if (boxDetection.averageRedGreen > t1.averageRedGreen) {
                                        return 1;
                                    } else if (boxDetection.averageRedGreen < t1.averageRedGreen) {
                                        return -1;
                                    } else {
                                        return 0;
                                    }
                                }
                            });

                            Location location = detections.get(0).location;
                            resultLocation.add(location);
                        }
                        resultAvaliable.open();
                    }
                }));

                resultAvaliable.block();
            }


            Log.d("Vision", "Size " + resultLocation.size());

            if (resultLocation.size() > 0) {
                if (resultLocation.get(0) == Location.LEFT) {
                    resultLocation.set(0, Location.RIGHT);
                } else if (resultLocation.get(0) == Location.RIGHT) {
                    resultLocation.set(0, Location.LEFT);
                }
                Log.d("Vision", "RESULT " + resultLocation.get(0).toString());
                return resultLocation.get(0);
            }
        }
        return Location.UNKNOWN;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        try {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

            //  Instantiate the Vuforia engine
            this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

            vuforia.enableConvertFrameToBitmap();
        } catch (Exception e) {

        }
    }
}

class BoxDetection {
    double averageRedGreen;
    Vision.Location location;

    public BoxDetection(Vision.Location location) {
        this.location = location;
    }
}

