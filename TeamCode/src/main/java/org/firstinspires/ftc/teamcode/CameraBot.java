package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;
import java.util.concurrent.BlockingQueue;

public class CameraBot extends PinchArmBot {
    public class Area extends Object{
        public int x;
        public int y;
        public int width;
        public int height;
    }

    Area box1 = new Area();
    Area box2 = new Area();
    Area box3 = new Area();

    public CameraBot(LinearOpMode opMode) {
        super(opMode);
    }

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AW3DaKr/////AAABmbYMj0zPp0oqll2pQvI8zaoN8ktPz319ETtFtBMP7b609q4wWm6yRX9OVwWnf+mXPgSC/fSdDI2uUp/69KTNAJ6Kz+sTx+9DG+mymW00Xm3LP7Xe526NP/lM1CIBsOZ2DJlQ2mqmObbDs5WR5HXyfopN12irAile/dEYkr3uIFnJ95P19NMdbiSlNQS6SNzooW0Nc8cBKWz91P020YDqC4dHSpbQvYeFgVp2VWZJC/uyvmE15nePzZ30Uq/n8pIeYWKh4+XR74RoRyabXMXFB6PZz7lgKdRMhhhBvQ5Eh21VxjE5h8ZhGw27K56XDPk63eczGTYP/FfeLvTuK4iKSNyqRLS/37kuxKn3t/dlkwv1";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Camera bot

        // setup frame queue
        vuforia.enableConvertFrameToBitmap();
        vuforia.setFrameQueueCapacity(5);

    }

    @Override
    public void init(HardwareMap ahwMap) {
        box1.x = 550;
        box1.y = 100;
        box1.width = 100;
        box1.height = 200;
        box2.x = 550;
        box2.y = 300;
        box2.width = 100;
        box2.height = 200;
        box3.x = 550;
        box3.y = 500;
        box3.width = 100;
        box3.height = 200;
        super.init(ahwMap);

        initVuforia();

    }

    @Override
    public void print(String message) {

    }

    final int NOSKYSTONE = 0;
    final int SKYSTONE1 = 1;
    final int SKYSTONE2 = 2;
    final int SKYSTONE3 = 3;

    protected void printAndSave(Bitmap bmp, int average, String label){
//        opMode.telemetry.log().add("Image %s with %d x %d and average RGB (%d, %d, %d)", label, bmp.getWidth(), bmp.getHeight(), Color.red(average), Color.green(average), Color.blue(average));
        opMode.telemetry.log().add("Image %s with %d x %d and average RGB #%02X #%02X #%02X", label, bmp.getWidth(), bmp.getHeight(), Color.red(average), Color.green(average), Color.blue(average));
        opMode.telemetry.update();
        opMode.sleep (3 * 1000);
        try (FileOutputStream out = new FileOutputStream(String.format("/sdcard/FIRST/ftc_%s.png", label))) {
            bmp.compress(Bitmap.CompressFormat.PNG, 100, out);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    public int detectSkystone() {
        BlockingQueue<VuforiaLocalizer.CloseableFrame> queue = vuforia.getFrameQueue();

        try {
            VuforiaLocalizer.CloseableFrame frame = queue.take();
            Image image = frame.getImage(0);
            opMode.telemetry.log().add("Got %d images from vuforia frame queue", frame.getNumImages());
            opMode.telemetry.log().add("Image #0 type = %d with %d x %d", image.getFormat(), image.getWidth(), image.getHeight());
            opMode.telemetry.log().add("Num Bytes = %d", image.getPixels().remaining());
            opMode.telemetry.update();
            Bitmap bmp = vuforia.convertFrameToBitmap(frame);
            printAndSave(bmp, getAverageRGB(bmp), "camera");
            frame.close();
            Bitmap b1, b2, b3;
            b1 = Bitmap.createBitmap(bmp, box1.x, box1.y, box1.width, box1.height);
            b2 = Bitmap.createBitmap(bmp, box2.x, box2.y, box2.width, box2.height);
            b3 = Bitmap.createBitmap(bmp, box3.x, box3.y, box3.width, box3.height);
            int c1, c2, c3;
            c1 = getAverageRGB(b1);
            c2 = getAverageRGB(b2);
            c3 = getAverageRGB(b3);
            printAndSave(b1, c1, "box1");
            printAndSave(b2, c2, "box2");
            printAndSave(b3, c3, "box3");
            int skystone = chooseSkystone(c1, c2, c3);
            return skystone;
        } catch (InterruptedException e) {
            print("Photo taken has been interrupted !");
            return NOSKYSTONE;
        }

    }

    public int chooseSkystone(int c1, int c2, int c3){

        if (c1 < c2 && c1 < c3) {
            return SKYSTONE1;
        } else if (c2 < c1 && c2 < c3) {
            return SKYSTONE2;
        } else if (c3 < c1 && c3 < c2) {
            return SKYSTONE3;
        }

        return SKYSTONE1;
    }

    protected int getAverageRGB(Bitmap bmp){

        int totalRed = 0;
        int totalGreen = 0;
        int totalBlue = 0;
        int width = bmp.getWidth();
        int height = bmp.getHeight();

        for (int x=0; x < width; x++) {
            for (int y=0; y < height; y++) {
                int pixel = bmp.getPixel(x, y);

                int red = Color.red(pixel);
                int green = Color.green(pixel);
                int blue = Color.blue(pixel);

                totalRed += red;
                totalGreen += green;
                totalBlue += blue;
            }
        }

        int averageRed = totalRed / (width * height);
        int averageGreen = totalGreen / (width * height);
        int averageBlue = totalBlue / (width * height);

        return Color.rgb(averageRed, averageGreen, averageBlue);
    }

}
