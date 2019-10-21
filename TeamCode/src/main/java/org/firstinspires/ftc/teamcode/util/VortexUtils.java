package org.firstinspires.ftc.teamcode.util;

import android.graphics.Bitmap;
import android.support.annotation.Nullable;
import android.util.Log;

import com.vuforia.CameraCalibration;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.Tool;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.RC;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.Arrays;

/**
 * Created by FIXIT on 16-11-20.
 */
public class VortexUtils {

    public final static int NOT_VISIBLE = 0;

    public final static int BEACON_BLUE_RED = 1;
    public final static int BEACON_RED_BLUE = 2;
    public final static int BEACON_ALL_BLUE = 3;
    public final static int BEACON_ALL_RED = 4;

    public final static int OBJECT_BLUE = 1;
    public final static int OBJECT_RED = 2;

    //hsv blue beacon range colours
    //DON'T CHANGE THESE NUMBERS
    public final static Scalar BEACON_BLUE_LOW = new Scalar(108, 0, 220);
    public final static Scalar BEACON_BLUE_HIGH = new Scalar(178, 255, 255);
    public final static Scalar BEACON_RED_LOW = new Scalar(200, 50, 0);
    public final static Scalar BEACON_RED_HIGH = new Scalar(255, 130, 255);
    
    public final static Scalar OTHER_BLUE_LOW = new Scalar(105, 120, 110);
    public final static Scalar OTHER_BLUE_HIGH = new Scalar(185, 255, 255);
    public final static Scalar OTHER_RED_LOW = new Scalar(222, 101, 192);
    public final static Scalar OTHER_RED_HIGH = new Scalar(47, 251, 255);

    public static Mat applyMask(Mat img, Scalar low, Scalar high) {

        Mat mask = new Mat(img.size(), CvType.CV_8UC3);

        if (high.val[0] < low.val[0]) {
            Scalar lowMed = new Scalar(255, high.val[1], high.val[2]);
            Scalar medHigh = new Scalar(0, low.val[1], low.val[2]);

            Mat maskLow = new Mat(img.size(), CvType.CV_8UC3);
            Mat maskHigh = new Mat(img.size(), CvType.CV_8UC3);

            Core.inRange(img, low, lowMed, maskLow);
            Core.inRange(img, medHigh, high, maskHigh);

            Core.bitwise_or(maskLow, maskHigh, mask);
        } else {
            Core.inRange(img, low, high, mask);
        }//else

        return mask;
    }

    public static int waitForBeaconConfig(Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camCal, long timeOut) {

        int config = NOT_VISIBLE;
        long beginTime = System.currentTimeMillis();
        while (config == NOT_VISIBLE && System.currentTimeMillis() - beginTime < timeOut && RC.l.opModeIsActive()) {
            config = getBeaconConfig(img, beacon, camCal);
            RC.l.idle();
        }//while

        return config;
    }

    /*
    NOTE: this method requires an image from an upside down camera
     */

    public static int getBeaconConfig(Bitmap bm) {
        if (bm != null) {

            //turning the corner pixel coordinates into a proper bounding box
            Mat image = OCVUtils.bitmapToMat(bm, CvType.CV_8UC3);

            //filtering out non-beacon-blue colours in HSV colour space
            Imgproc.cvtColor(image, image, Imgproc.COLOR_RGB2HSV_FULL);

            //get filtered mask
            //if pixel is within acceptable blue-beacon-colour range, it's changed to white.
            //Otherwise, it's turned to black
            Mat blueMask = new Mat();
            Mat redMask = new Mat();

            Core.inRange(image, BEACON_BLUE_LOW, BEACON_BLUE_HIGH, blueMask);
            Core.inRange(image, BEACON_RED_LOW, BEACON_RED_HIGH, redMask);
            Moments blueMmnts = Imgproc.moments(blueMask, true);
            Moments redMmnts = Imgproc.moments(redMask, true);

            //calculating centroid of the resulting binary mask via image moments
            Log.i("CentroidBlueX", "" + (( blueMmnts.get_m00())));
            Log.i("CentroidBlueY", "" + (blueMmnts.get_m00()));
            Log.i("CentroidRedX", "" + (redMmnts.get_m00()));
            Log.i("CentroidRedY", "" + (redMmnts.get_m00()));
            Log.i("Momentus", (blueMmnts.get_m00() / redMmnts.get_m00()) + "");


            //Note: for some reason, we end up with a image that is rotated 90 degrees
            //if centroid is in the bottom half of the image, the blue beacon is on the left
            //if the centroid is in the top half, the blue beacon is on the right
            if ((blueMmnts.get_m01() / blueMmnts.get_m00()) < image.rows() / 2) {
                return VortexUtils.BEACON_BLUE_RED;
            } else {
                return VortexUtils.BEACON_RED_BLUE;
            }//else
        }//if

        return VortexUtils.NOT_VISIBLE;
    }//getBeaconConfig

    public static int doubleCheckBeaconConfig(Bitmap bm) {

        if (bm != null) {
            //turning the corner pixel coordinates into a proper bounding box
            Mat image = OCVUtils.bitmapToMat(bm, CvType.CV_8UC3);

            //filtering out non-beacon-blue colours in HSV colour space
            Imgproc.cvtColor(image, image, Imgproc.COLOR_RGB2HSV_FULL);

            //get filtered mask
            //if pixel is within acceptable blue-beacon-colour range, it's changed to white.
            //Otherwise, it's turned to black
            Mat blueMask = new Mat();
            Mat redMask = new Mat();

            Core.inRange(image, BEACON_BLUE_LOW, BEACON_BLUE_HIGH, blueMask);
            Core.inRange(image, BEACON_RED_LOW, BEACON_RED_HIGH, redMask);
            Moments blueMmnts = Imgproc.moments(blueMask, true);
            Moments redMmnts = Imgproc.moments(redMask, true);

            //calculating centroid of the resulting binary mask via image moments
            Log.i("CentroidBlueX", "" + (( blueMmnts.get_m00())));
            Log.i("CentroidBlueY", "" + (blueMmnts.get_m00()));
            Log.i("CentroidRedX", "" + (redMmnts.get_m00()));
            Log.i("CentroidRedY", "" + (redMmnts.get_m00()));
            Log.i("Momentus", (blueMmnts.get_m00() / redMmnts.get_m00()) + "");

    //            checking if blue either takes up the majority of the image (which means the beacon is all blue)
    //            or if there's barely any blue in the image (which means the beacon is all red or off)
            if (blueMmnts.get_m00() / redMmnts.get_m00() > 3) {
                return VortexUtils.BEACON_ALL_BLUE;
            } else if (blueMmnts.get_m00() / redMmnts.get_m00() < 0.1) {
                return VortexUtils.BEACON_ALL_RED;
            }//elseif

            //Note: for some reason, we end up with a image that is rotated 90 degrees
            //if centroid is in the bottom half of the image, the blue beacon is on the left
            //if the centroid is in the top half, the blue beacon is on the right
            if ((blueMmnts.get_m01() / blueMmnts.get_m00()) < image.rows() / 2) {
                return VortexUtils.BEACON_BLUE_RED;
            } else {
                return VortexUtils.BEACON_RED_BLUE;
            }//else
        }//if

        return VortexUtils.NOT_VISIBLE;
    }

    public static int getBeaconConfig(Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camCal) {

        OpenGLMatrix pose = beacon.getRawPose();

        if (pose != null && img != null && img.getPixels() != null) {

            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);

            rawPose.setData(poseData);

            //calculating pixel coordinates of beacon corners
            float[][] corners = new float[4][2];

            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 276, 0)).getData(); //upper left of beacon
            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 276, 0)).getData(); //upper right of beacon
            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, -92, 0)).getData(); //lower right of beacon
            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, -92, 0)).getData(); //lower left of beacon

            //getting camera image...
            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());
            //Hi

            //turning the corner pixel coordinates into a proper bounding box
            Mat crop = OCVUtils.bitmapToMat(bm, CvType.CV_8UC3);
            float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
            float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
            float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
            float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));


            //make sure our bounding box doesn't go outside of the image
            //OpenCV doesn't like that...
            x = Math.max(x, 0);
            y = Math.max(y, 0);
            width = (x + width > crop.cols())? crop.cols() - x : width;
            height = (y + height > crop.rows())? crop.rows() - y : height;

            //cropping bounding box out of camera image
            final Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));

            //filtering out non-beacon-blue colours in HSV colour space
            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);

            //get filtered mask
            //if pixel is within acceptable blue-beacon-colour range, it's changed to white.
            //Otherwise, it's turned to black
            Mat mask = new Mat();

            Core.inRange(cropped, BEACON_BLUE_LOW, BEACON_BLUE_HIGH, mask);
            Moments mmnts = Imgproc.moments(mask, true);

            //calculating centroid of the resulting binary mask via image moments
            Log.i("CentroidX", "" + ((mmnts.get_m10() / mmnts.get_m00())));
            Log.i("CentroidY", "" + ((mmnts.get_m01() / mmnts.get_m00())));

            //Note: for some reason, we end up with a image that is rotated 90 degrees
            //if centroid is in the bottom half of the image, the blue beacon is on the left
            //if the centroid is in the top half, the blue beacon is on the right
            if ((mmnts.get_m01() / mmnts.get_m00()) < cropped.rows() / 2) {
                return VortexUtils.BEACON_RED_BLUE;
            } else {
                return VortexUtils.BEACON_BLUE_RED;
            }//else
        }//if

        return VortexUtils.NOT_VISIBLE;
    }//getBeaconConfig



    public static int isBlueOrRed(Image img) {

        if (img != null && img.getPixels() != null) {

            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            //turning the corner pixel coordinates into a proper bounding box
            Mat analyse = OCVUtils.bitmapToMat(bm, CvType.CV_8UC3);

            Mat maskBlue = applyMask(analyse.clone(), OTHER_BLUE_LOW, OTHER_BLUE_HIGH);

            Mat maskRed = applyMask(analyse.clone(), OTHER_RED_LOW, OTHER_RED_HIGH);

            if (Imgproc.moments(maskBlue).get_m00() > Imgproc.moments(maskRed).get_m00()) {
                return OBJECT_BLUE;
            } else {
                return OBJECT_RED;
            }//else

        }//if

        return NOT_VISIBLE;
    }//isBlueOrRed

    @Nullable
    public static Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {

        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }//if
        }//for

        return null;
    }

    //this assumes the horizontal axis is the y-axis since the phone is vertical
    //robot angle is relative to "parallel with the beacon wall"
    public static VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF(
                (float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))),
                trans.get(1) + offWall.get(1),
                (float) (-trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle)))
        );
    }

    public static VectorF navOffWall2(VectorF trans, double robotAngle, VectorF offWall){
        double theta = Math.toDegrees(Math.atan2(offWall.get(0), offWall.get(2)));

        return new VectorF(
                (float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))),
                trans.get(1),
                (float) (-trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle)))
        );
    }
}
