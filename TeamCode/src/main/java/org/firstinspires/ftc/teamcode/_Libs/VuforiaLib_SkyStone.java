package org.firstinspires.ftc.teamcode._Libs;

import android.graphics.Bitmap;
import android.graphics.Point;
import android.graphics.Rect;
import android.graphics.RectF;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Created by phanau on 9/14/18.
 */


/**
 * This library encapsulates the code needed by OpModes using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 *
 * Vuforia uses the phone's camera to inspect its surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * IMPORTANT: In order to use this Library, you can either obtain your own Vuforia license key as
 * is explained below or use the one supplied by this library.
 */

// we need to break into the "impl" class to get access to its close() function so we can
// really shut it down and get back control of the camera
class myVuforiaLocalizerImpl extends VuforiaLocalizerImpl
{
    public myVuforiaLocalizerImpl(VuforiaLocalizer.Parameters parameters)
    {
        super(parameters);
    }
    public void _close()  { close(); }
}


public class VuforiaLib_SkyStone implements HeadingSensor, LocationSensor {

    myVuforiaLocalizerImpl vuforia;
    VuforiaTrackables targetsSkyStone = null;
    List<VuforiaTrackable> allTrackables;
    OpenGLMatrix lastLocation = null;
    OpMode mOpMode;

    BlockingQueue<VuforiaLocalizer.CloseableFrame> mFrameQueue;
    VuforiaLocalizer.CloseableFrame mCF;

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    public void init(OpMode opMode, String licenseKey) {

        // remember this so we can do telemetry output
        mOpMode = opMode;

        /**
         * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
         * the camera monitor feedback; if no camera monitor feedback is desired, use the parameterless
         * constructor instead. We also indicate which camera on the RC that we wish to use. For illustration
         * purposes here, we choose the back camera; for a competition robot, the front camera might
         * prove to be more convenient.
         *
         * Note that in addition to indicating which camera is in use, we also need to tell the system
         * the location of the phone on the robot; see phoneLocationOnRobot below.
         *
         * IMPORTANT: You should obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is valid and will function, but better to get your own.
         * Vuforia will not load without a valid license being provided. Vuforia 'Development' license
         * keys, which is what is needed here, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Valid Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string form of the key from the Vuforia web site
         * and paste it in to your code as the value of the 'vuforiaLicenseKey' field of the
         * {@link Parameters} instance with which you initialize Vuforia.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey =
                (licenseKey != null && licenseKey.length() > 0) ? licenseKey :
                        "ARf809H/////AAAAGRswBQwUCUJ5nqfgZxGbDEQ8oO7YP5GdnbReYr8ZHinqQ74OsP7UdOxNZJDmhaF2OeGD20jpSexpr2CcXGSGuHXNB2p9Z6zUNLDTfEggL+yg4ujefoqdkSpCqZf1medpwh3KXcK76FcfSJuqEudik2PC6kQW/cqJXnnHofVrrDTzJmWMnK3hlqTMjig81DEPMAHbRnA5wn7Eu0irnmqqboWyOlQ0xTF+P4LVuxaOUFlQC8zPqkr1Gvzvix45paWtyuLCnS9YDWMvI1jIM4giMrTRCT0lG8F+vkuKMiK647KJp9QIsFdWQ0ecQhau3ODNQ03pcTzprVN72b9VObpv6FNBpjGKRAcA59xlZiM2l6fc";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = new myVuforiaLocalizerImpl(parameters); // ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data sets that for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
         * example "StonesAndChips", datasets can be found in in this project in the
         * documentation directory.
         */

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);


        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        /**
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = phoneLocationOnRobot  maps   phone coords -> robot coords
         * P = tracker.getPose()     maps   image target coords -> phone coords
         * L = redTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()              maps   robot coords -> phone coords
         * P.inverted()              maps   phone coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        // get access to video frames so we can do other processing like looking for scene objects
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB format for the image
        vuforia.setFrameQueueCapacity(1);
        mFrameQueue = vuforia.getFrameQueue();
        mCF = null;
    }

    public void lightOn(boolean bLightOn) {
        // turn on the flashlight
        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(bLightOn);
    }

    public void start()
    {
        /** Start tracking the data sets we care about. */
        targetsSkyStone.activate();
    }

    public void loop(boolean bTelemetry)
    {
        lastLocation = null;    // reset each time so we can tell if we currently have any target visible

        for (VuforiaTrackable trackable : allTrackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            if (bTelemetry && ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible())
                mOpMode.telemetry.addData(trackable.getName(), "Visible");    //

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }

        /**
         * Provide feedback as to where the robot was last located (if we know).
         */
        if (bTelemetry) {
            if (haveLocation())
                mOpMode.telemetry.addData("Position:", formatPosition(lastLocation));
            else
                mOpMode.telemetry.addData("Position:", "Unknown");
            if (haveHeading())
                mOpMode.telemetry.addData("Orientation:", formatOrientation(lastLocation));
            else
                mOpMode.telemetry.addData("Orientation:", "Unknown");
        }
    }

    public void stop()
    {
        /** Stop tracking the data sets we care about. */
        targetsSkyStone.deactivate();

        // close down Vuforia NOW so other code can use the camera
        vuforia._close();
    }

    // return lastLocation matrix (may be null)
    public OpenGLMatrix getLastLocation()
    {
        return lastLocation;
    }

    public VectorF getFieldPosition()
    {
        if (lastLocation != null)
            return lastLocation.getTranslation();
        else
            return null;
    }

    public Orientation getOrientation(AxesReference ref, AxesOrder order, AngleUnit unit)
    {
        if (lastLocation != null)
            return Orientation.getOrientation(lastLocation, ref, order, unit);
        else
            return null;
    }

    public Orientation getOrientation()
    {
        return this.getOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }

    public float getPitch()
    {
        return this.getOrientation().firstAngle;
    }

    public float getRoll()
    {
        return this.getOrientation().secondAngle;
    }

    public float getYaw()
    {
        return this.getOrientation().thirdAngle;    // -180 .. 0 .. +180
    }

    // implements HeadingSensor interface
    public float getHeading()
    {
        return getYaw();
    }
    public boolean haveHeading()
    {
        return (lastLocation != null);
    }
    public void setHeadingOffset(float offset) {}   // not used; Vuforia headings are field-absolute

    // implements LocationSensor interface
    public VectorF getLocation() { return getFieldPosition(); }
    public boolean haveLocation() { return (lastLocation != null);}       // is there valid location data?

    /**
     * Some simple utilities that extract information from a transformation matrix
     * and format it in a form palatable to a human being.
     * For sanity's sake, display translations in inches rather than mm.
     */

    String formatPosition(OpenGLMatrix transformationMatrix) {
        //return transformationMatrix.formatAsTransform();
        VectorF translation = transformationMatrix.getTranslation();
        translation.multiply(1.0f/25.4f);       // convert from mm to inches
        return String.format("%s inches", translation.toString());
    }
    String formatOrientation(OpenGLMatrix transformationMatrix) {
        //return transformationMatrix.formatAsTransform();
        Orientation orientation = Orientation.getOrientation(transformationMatrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return String.format("%s", orientation.toString());
    }


    // get a bitmap from Vuforia - these are our own versions from 2017 code --
    // they don't use Vuforia convertFrameToBitmap function, which are essentially the same as these but don't do scaling or cropping as we do.

    // get entire image downsampled by an integer factor (e.g. 4 gives an image 1/4 the height and width of what the camera delivered)
    public Bitmap getBitmap(int sample) {
        return getBitmap(new RectF(0,0,1,1), sample);
    }

    // get entire image scaled by the given scale factor
    // (e.g. 4 gives an image 1/4 the height and width of what the camera delivered)
    public Bitmap getBitmap(float scale) {
        return getBitmap(new RectF(0,0,1,1), scale);
    }

    // get entire image scaled to the given width and height (result will be distorted if you don't get this right)
    public Bitmap getBitmap(int width, int height) {
        return getBitmap(new RectF(0,0,1,1), 0, width, height);
    }

    // get a cropped portion of the camera image downsampled by an integer factor
    // (e.g. sample=4 gives an image 1/4 the height and width of the cropped portion)
    public Bitmap getBitmap(RectF rect, int sample) {
        return getBitmap(rect, 1.0f/sample, 0, 0);
    }

    // get a cropped portion of the camera image scaled by a given factor
    // (e.g. scale=0.25 gives an image 1/4 the height and width of the cropped portion)
    public Bitmap getBitmap(RectF rect, float scale) {
        return getBitmap(rect, scale, 0, 0);
    }

    // get a cropped portion of the camera image scaled to the given width and height (result will be distorted if you don't get this right)
    public Bitmap getBitmap(RectF rect, int width, int height) {
        return getBitmap(rect, 0, width, height);
    }

    // this function handles all the various ways of describing the final output size
    // rect is the portion of the source image you want, normalized 0..1 in both axes
    private Bitmap getBitmap(RectF rect, float scale, int height, int width) {
        try {
            VuforiaLocalizer.CloseableFrame frame = mFrameQueue.take();
            int img = 0;
            for (; img < frame.getNumImages(); img++) {
                //telemetry.addData("Image format " + img, frame.getImage(img).getFormat());
                if (frame.getImage(img).getFormat() == PIXEL_FORMAT.RGB565) break;
            }

            if (img == frame.getNumImages()) throw new IllegalArgumentException("Incorrect format");

            // get the Image with the correct format and extract its data
            Image image = frame.getImage(img);
            ByteBuffer byteBuffer = image.getPixels();

            int h = image.getHeight();    // height of src data
            int w = image.getWidth();     // width of src data

            // convert the data to a Bitmap
            byteBuffer.rewind();
            Bitmap bitmap = Bitmap.createBitmap(w, h, Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(byteBuffer);

            // crop the Bitmap if requested
            Bitmap bitmapCropped = bitmap;
            int ch = Math.round(h*rect.height());    // height of cropped src data
            int cw = Math.round(w*rect.width());     // width of cropped src data
            if (ch < h || cw < w) {
                bitmapCropped = Bitmap.createBitmap(bitmap,
                        Math.max(0, Math.round(w*rect.left)), Math.max(0, Math.round(h*rect.top)), cw, ch);
                h = ch; w = cw;
            }

            // get requested output size in one of several ways
            int dstW = cw;
            int dstH = ch;
            if (width>0 && height>0) {
                dstW = width;
                dstH = height;
            }
            else if (scale > 0) {
                dstW = (int)(cw*scale);
                dstH = (int)(ch*scale);
            }

            // scale the (possibly cropped) result bitmap to the requested size
            Bitmap bitmapScaled = Bitmap.createScaledBitmap(bitmapCropped, dstW, dstH, true);

            frame.close();

            return bitmapScaled;
        }
        catch (Exception e) {}

        return null;
    }

    // get the size of the bitmap Vuforia would return
    public Point getBitmapSize() {
        try {
            VuforiaLocalizer.CloseableFrame frame = mFrameQueue.take();
            int img = 0;
            for (; img < frame.getNumImages(); img++) {
                //telemetry.addData("Image format " + img, frame.getImage(img).getFormat());
                if (frame.getImage(img).getFormat() == PIXEL_FORMAT.RGB565) break;
            }

            if (img == frame.getNumImages()) throw new IllegalArgumentException("Incorrect format");

            // get the Image with the correct format and extract its data
            Image image = frame.getImage(img);

            int h = image.getHeight();    // height of src data
            int w = image.getWidth();     // width of src data

            return new Point(w, h);
        } catch (Exception e) {
            return null;
        }
    }


    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod = null;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public TFObjectDetector initTfod(OpMode opmode) {
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {

            int tfodMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
            return tfod;
        }
        return null;
    }

    public TFObjectDetector getTfod() {
        return tfod;
    }
}
