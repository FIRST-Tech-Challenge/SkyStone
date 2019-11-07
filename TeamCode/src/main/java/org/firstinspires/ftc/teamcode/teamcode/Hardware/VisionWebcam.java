package org.firstinspires.ftc.teamcode.teamcode.Hardware;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.ArrayList;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

public class VisionWebcam {

    LinearOpMode opMode;
    private VuforiaLocalizer vuforia;

    public static final String vuKey =
            "AdzMYbL/////AAABmflzIV+frU0RltL/ML+2uAZXgJiI" +
                    "Werfe92N/AeH7QsWCOQqyKa2G+tUDcgvg8uE8QjHeBZPcpf5hAwlC5qCfvg76eBoaa2b" +
                    "MMZ73hmTiHmr9fj3XmF4LWWZtDC6pWTFrzRAUguhlvgnck6Y4jjM16Px5TqgWYuWnpcxNM" +
                    "HMyOXdnHLlyysyE64PVzoN7hgMXgbi2K8+pmTXvpV2OeLCag8fAj1Tgdm/kKGr0TX86aQsC2" +
                    "RVjToZXr9QyAeyODi4l1KEFmGwxEoteNU8yqNbBGkPXGh/+IIm6/s/KxCJegg8qhxZDgO8110F" +
                    "RzwA5a6EltfxAMmtO0G8BB9SSkApxkcSzpyI0k2LxWof2YZG6x4H";

    public void initVision(LinearOpMode opMode) {
        this.opMode = opMode;

        // variable allows image to show up on robot controller phone
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // original vuforia license key
        parameters.vuforiaLicenseKey = vuKey;
        // hardware mapping of webcam device
        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        // start vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // set RGB format to 565
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        // allowing the frame to only be 3 images at a time
        vuforia.setFrameQueueCapacity(3);
        opMode.telemetry.addLine("Vision init");
    }
    public Bitmap getBitmap() throws InterruptedException{
        // method to actually capture frame
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        Image rgb = frame.getImage(1);

        long numImages = frame.getNumImages();

        // go through all images taken in frame and find ones that match correct format
        for (int i = 0; i < numImages; i++) {
            int fmt = frame.getImage(i).getFormat();

            if (fmt == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;

            }
            else {
                opMode.telemetry.addLine("Didn't find correct rgb format");
                opMode.telemetry.update();

            }

        }

        // create bitmap
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        frame.close();

        opMode.telemetry.addLine("Got Bitmap");
        opMode.telemetry.addData("width", rgb.getWidth());
        opMode.telemetry.addData("height", rgb.getHeight());
        opMode.telemetry.update();

        opMode.sleep(500);

        return bm;
    }

    public String senseBlue(LinearOpMode opMode) throws InterruptedException {

            Bitmap bitmap = getBitmap();
            ArrayList<Integer> StoneX = new ArrayList<Integer>();

            String pos = "";
            int stonexAvg = 0;

            // top left = (0,0)

            // scan 3 columns
            for (int colNum = bitmap.getWidth() / 2; colNum < bitmap.getWidth() ; colNum++) {

                for (int rowNum = bitmap.getHeight() / 2; rowNum < bitmap.getHeight(); rowNum++) {
                    int pixel = bitmap.getPixel(colNum, rowNum);

                    // receive R, G, and B values for each pixel
                    int redPixel = red(pixel);
                    int greenPixel = green(pixel);
                    int bluePixel = blue(pixel);

                    /*opMode.telemetry.addData("Red", redPixel);
                    opMode.telemetry.addData("Green", greenPixel);
                    opMode.telemetry.addData("Blue", bluePixel);
                    opMode.telemetry.update();*/
                    // only add x-coordinates of black pixels to list

                    if (redPixel < 30 && greenPixel < 30 && bluePixel < 30) {
                        StoneX.add(colNum);
                    }


                }
            }


            // get sum of all yellow pixels' x coordinates
            for (int x : StoneX) {
                stonexAvg += x;
            }

             stonexAvg /= StoneX.size();


            // get average x-coordinate value of all yellow pixels

            opMode.telemetry.addData("AVG X = ", stonexAvg);
            opMode.telemetry.update();

            // use constants (235, 500) to determine which third of the image cube is in


            if (stonexAvg < 500) {
                pos = "left";
            } else if (stonexAvg >600) {
                pos = "right";
            } else {
                pos = "center";
            }

            opMode.telemetry.addData("Position", pos);
            opMode.telemetry.update();

        return pos;
    }

    public String senseRed(LinearOpMode opMode) throws InterruptedException {

        Bitmap bitmap = getBitmap();
        ArrayList<Integer> StoneX = new ArrayList<Integer>();

        String pos = "";
        int stonexAvg = 0;

        // top left = (0,0)
        while (opMode.opModeIsActive()) {


            // scan 3 columns
            for (int colNum = bitmap.getWidth() / 2; colNum < bitmap.getWidth() ; colNum++) {

                for (int rowNum = bitmap.getHeight()/2; rowNum < bitmap.getHeight(); rowNum++){
                    int pixel = bitmap.getPixel(colNum, rowNum);

                    // receive R, G, and B values for each pixel
                    int redPixel = red(pixel);
                    int greenPixel = green(pixel);
                    int bluePixel = blue(pixel);

                   /* opMode.telemetry.addData("Red", redPixel);
                    opMode.telemetry.addData("Green", greenPixel);
                    opMode.telemetry.addData("Blue", bluePixel);
                    opMode.telemetry.update();*/
                    // only add x-coordinates of black pixels to list

                    if (bluePixel < 50) {
                        StoneX.add(colNum);
                    }


                }
            }


            // get sum of all yellow pixels' x coordinates
            for (int x : StoneX) {
                stonexAvg += x;
            }

            stonexAvg /= StoneX.size();


            // get average x-coordinate value of all yellow pixels

            opMode.telemetry.addData("AVG X = ", stonexAvg);
            opMode.telemetry.update();

            // use constants (235, 500) to determine which third of the image cube is in


            if (stonexAvg < 500) {
                pos = "left";
            } else if (stonexAvg > 600) {
                pos = "right";
            } else {
                pos = "center";
            }

            opMode.telemetry.addData("Position", pos);
            opMode.telemetry.update();
        }

        return pos;
    }


    public Bitmap vufConvertToBitmap(Frame frame) { return vuforia.convertFrameToBitmap(frame); }

}
