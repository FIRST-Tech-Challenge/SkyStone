package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

public class VuforiaSensorSkystone {

    public VuforiaLocalizer vuforiaLocalizer;

    public VuforiaLocalizer.Parameters parameters;

    public VuforiaTrackables visionTargets;

    public VuforiaTrackable targetSkystone;
    public VuforiaTrackable targetRedPerimeterTgt1;
    public VuforiaTrackable targetRedPerimeterTgt2;
    public VuforiaTrackable targetFrontPerimeterTgt1;
    public VuforiaTrackable targetFrontPerimeterTgt2;
    public VuforiaTrackable targetBluePerimeterTgt1;
    public VuforiaTrackable targetBluePerimeterTgt2;
    public VuforiaTrackable targetRearPerimeterTgt1;
    public VuforiaTrackable targetRearPerimeterTgt2;

    public VuforiaTrackableDefaultListener listenerSkystone;
    public VuforiaTrackableDefaultListener listenerRedPerimeterTgt1;
    public VuforiaTrackableDefaultListener listenerRedPerimeterTgt2;
    public VuforiaTrackableDefaultListener listenerFrontPerimeterTgt1;
    public VuforiaTrackableDefaultListener listenerFrontPerimeterTgt2;
    public VuforiaTrackableDefaultListener listenerBluePerimeterTgt1;
    public VuforiaTrackableDefaultListener listenerBluePerimeterTgt2;
    public VuforiaTrackableDefaultListener listenerRearPerimeterTgt1;
    public VuforiaTrackableDefaultListener listenerRearPerimeterTgt2;

    public OpenGLMatrix lastKnownLocationSkystone;
    public OpenGLMatrix lastKnownLocationRedPerimeterTgt1;
    public OpenGLMatrix lastKnownLocationRedPerimeterTgt2;
    public OpenGLMatrix lastKnownLocationFrontPerimeterTgt1;
    public OpenGLMatrix lastKnownLocationFrontPerimeterTgt2;
    public OpenGLMatrix lastKnownLocationBluePerimeterTgt1;
    public OpenGLMatrix lastKnownLocationBluePerimeterTgt2;
    public OpenGLMatrix lastKnownLocationRearPerimeterTgt1;
    public OpenGLMatrix lastKnownLocationRearPerimeterTgt2;

    public OpenGLMatrix phoneLocation;

    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    public VuforiaSensorSkystone(Telemetry telemetry, HardwareMap hardWareMap) {

        this.telemetry = telemetry;
        this.hardwareMap = hardWareMap;

        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = "ATUNNu//////AAABmU6BPERoN0USgSQzxPQZ8JYg9RVnQhKO6YEHbNnOhkfL/iNrji3x9vzFkKsBgVzWgwH72G6eXpb3VCllKTrt1cD3gvQXZ48f+5EN43eYUQ3nuP3943NZB822XzV1djS3s6wDdaiS20PErO5K7lZUGyf9Z4Tb2TliOXv/ZoxUvwNQ/ndRjN344G0TAo8PUja0V3x2WKk+mCJavoZIgmOqgaitgmg5jim/aWBL2yk0a/QpqbP87KQfGn69zpisDBc98xdGPdSFj9ENkU9WTMem9UgnOFPgpdrHV5Zr5IpQH1jxLZIvwGuKOT97npm54kIvnJM0dzhBVA+s95JA3cxyac5ArHUYVtDePwlExuekZy9l"; // Insert your own key here

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        parameters.cameraName = hardWareMap.get(WebcamName.class, "Webcam 1");

        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("Skystone");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 9);

        // Setup the target to be tracked
        targetSkystone = visionTargets.get(0);
        targetSkystone.setName("Skystone");
        targetSkystone.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        targetRedPerimeterTgt1 = visionTargets.get(5); // 0 corresponds to the BluePerimeter target
        targetRedPerimeterTgt1.setName("RedP1");
        targetRedPerimeterTgt1.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        targetRedPerimeterTgt2 = visionTargets.get(6);
        targetRedPerimeterTgt2.setName("RedP2");
        targetRedPerimeterTgt2.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        targetFrontPerimeterTgt1 = visionTargets.get(7);
        targetFrontPerimeterTgt1.setName("FP1");
        targetFrontPerimeterTgt1.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        targetFrontPerimeterTgt2 = visionTargets.get(8);
        targetFrontPerimeterTgt2.setName("FP2");
        targetFrontPerimeterTgt2.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        targetBluePerimeterTgt1 = visionTargets.get(9);
        targetBluePerimeterTgt1.setName("BP1");
        targetBluePerimeterTgt1.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        targetBluePerimeterTgt2 = visionTargets.get(10);
        targetBluePerimeterTgt2.setName("BP2");
        targetBluePerimeterTgt2.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        targetRearPerimeterTgt1 = visionTargets.get(11);
        targetRearPerimeterTgt1.setName("RP1");
        targetRearPerimeterTgt1.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        targetRearPerimeterTgt2 = visionTargets.get(12);
        targetRearPerimeterTgt2.setName("RP2");
        targetRearPerimeterTgt2.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        //Set phone location on robot
        phoneLocation = createMatrix(0, 0, 0, 0, 0, 0);

        // Setup listener and inform it of phone information
        listenerSkystone = (VuforiaTrackableDefaultListener) targetSkystone.getListener();
        listenerSkystone.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        listenerRedPerimeterTgt1 = (VuforiaTrackableDefaultListener) targetRedPerimeterTgt1.getListener();
        listenerRedPerimeterTgt1.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        listenerRedPerimeterTgt2 = (VuforiaTrackableDefaultListener) targetRedPerimeterTgt2.getListener();
        listenerRedPerimeterTgt2.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        listenerFrontPerimeterTgt1 = (VuforiaTrackableDefaultListener) targetFrontPerimeterTgt1.getListener();
        listenerFrontPerimeterTgt1.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        listenerFrontPerimeterTgt2 = (VuforiaTrackableDefaultListener) targetFrontPerimeterTgt2.getListener();
        listenerFrontPerimeterTgt2.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        listenerBluePerimeterTgt1 = (VuforiaTrackableDefaultListener) targetBluePerimeterTgt1.getListener();
        listenerBluePerimeterTgt1.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        listenerBluePerimeterTgt2 = (VuforiaTrackableDefaultListener) targetBluePerimeterTgt2.getListener();
        listenerBluePerimeterTgt2.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        listenerRearPerimeterTgt1 = (VuforiaTrackableDefaultListener) targetRearPerimeterTgt1.getListener();
        listenerRearPerimeterTgt1.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        listenerRearPerimeterTgt2 = (VuforiaTrackableDefaultListener) targetRearPerimeterTgt2.getListener();
        listenerRearPerimeterTgt2.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        lastKnownLocationSkystone = createMatrix(0, 0, 0, 0, 0, 0);
        lastKnownLocationRedPerimeterTgt1 = createMatrix(0, 0, 0, 0, 0, 0);
        lastKnownLocationRedPerimeterTgt2 = createMatrix(0, 0, 0, 0, 0, 0);
        lastKnownLocationFrontPerimeterTgt1 = createMatrix(0, 0, 0, 0, 0, 0);
        lastKnownLocationFrontPerimeterTgt2 = createMatrix(0, 0, 0, 0, 0, 0);
        lastKnownLocationBluePerimeterTgt1 = createMatrix(0, 0, 0, 0, 0, 0);
        lastKnownLocationBluePerimeterTgt2 = createMatrix(0, 0, 0, 0, 0, 0);
        lastKnownLocationRearPerimeterTgt1 = createMatrix(0, 0, 0, 0, 0, 0);
        lastKnownLocationRearPerimeterTgt2 = createMatrix(0, 0, 0, 0, 0, 0);
    }

    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    private String formatMatrix(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }

    public void activate() {
        visionTargets.activate();
    }

    public void loop() {
        // Ask the listener for the latest information on where the robot is
        OpenGLMatrix latestLocationSkystone = listenerSkystone.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationRedPerimeterTgt1 = listenerRedPerimeterTgt1.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationRedPerimeterTgt2 = listenerRedPerimeterTgt2.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationFrontPerimeterTgt1 = listenerFrontPerimeterTgt1.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationFrontPerimeterTgt2 = listenerFrontPerimeterTgt2.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationBluePerimeterTgt1 = listenerBluePerimeterTgt1.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationBluePerimeterTgt2 = listenerBluePerimeterTgt2.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationRearPerimeterTgt1 = listenerRearPerimeterTgt1.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationRearPerimeterTgt2 = listenerRearPerimeterTgt2.getUpdatedRobotLocation();

        // The listener will sometimes return null, so we check for that to prevent errors
        if (latestLocationSkystone != null)
            lastKnownLocationSkystone = latestLocationSkystone;
        if (latestLocationRedPerimeterTgt1 != null)
            lastKnownLocationRedPerimeterTgt1 = latestLocationRedPerimeterTgt1;
        if (latestLocationRedPerimeterTgt2 != null)
            lastKnownLocationRedPerimeterTgt2 = latestLocationRedPerimeterTgt2;
        if (latestLocationFrontPerimeterTgt1 != null)
            lastKnownLocationFrontPerimeterTgt1 = latestLocationFrontPerimeterTgt1;
        if (latestLocationFrontPerimeterTgt2 != null)
            lastKnownLocationFrontPerimeterTgt2 = latestLocationFrontPerimeterTgt2;
        if (latestLocationBluePerimeterTgt1 != null)
            lastKnownLocationBluePerimeterTgt1 = latestLocationBluePerimeterTgt1;
        if (latestLocationBluePerimeterTgt2 != null)
            lastKnownLocationBluePerimeterTgt2 = latestLocationBluePerimeterTgt2;
        if (latestLocationRearPerimeterTgt1 != null)
            lastKnownLocationRearPerimeterTgt1 = latestLocationRearPerimeterTgt1;
        if (latestLocationRearPerimeterTgt2 != null)
            lastKnownLocationRearPerimeterTgt2 = latestLocationRearPerimeterTgt2;

        // Send information about whether the target is visible, and where the robot is
        if (listenerSkystone.isVisible()) {
            telemetry.addData("Tracking", targetSkystone.getName());
            telemetry.addData("position", lastKnownLocationSkystone);
        }
        if (listenerRedPerimeterTgt1.isVisible()) {
            telemetry.addData("Tracking", targetRedPerimeterTgt1.getName());
            telemetry.addData("position", lastKnownLocationRedPerimeterTgt1);
        }
        if (listenerRedPerimeterTgt2.isVisible()) {
            telemetry.addData("Tracking", targetRedPerimeterTgt2.getName());
            telemetry.addData("position", lastKnownLocationRedPerimeterTgt2);
        }
        if (listenerFrontPerimeterTgt1.isVisible()) {
            telemetry.addData("Tracking", targetFrontPerimeterTgt1.getName());
            telemetry.addData("position", lastKnownLocationFrontPerimeterTgt1);
        }
        if (listenerFrontPerimeterTgt2.isVisible()) {
            telemetry.addData("Tracking", targetFrontPerimeterTgt2.getName());
            telemetry.addData("position", lastKnownLocationFrontPerimeterTgt2);
        }
        if (listenerBluePerimeterTgt1.isVisible()) {
            telemetry.addData("Tracking", targetBluePerimeterTgt1.getName());
            telemetry.addData("position", lastKnownLocationBluePerimeterTgt1);
        }
        if (listenerBluePerimeterTgt2.isVisible()) {
            telemetry.addData("Tracking", targetBluePerimeterTgt2.getName());
            telemetry.addData("position", lastKnownLocationBluePerimeterTgt2);
        }
        if (listenerRearPerimeterTgt1.isVisible()) {
            telemetry.addData("Tracking", targetRearPerimeterTgt1.getName());
            telemetry.addData("position", lastKnownLocationRearPerimeterTgt1);
        }
        if (listenerRearPerimeterTgt2.isVisible()) {
            telemetry.addData("Tracking", targetRearPerimeterTgt2.getName());
            telemetry.addData("position", lastKnownLocationRearPerimeterTgt2);
        }
    }

    public float[] getTranslation(OpenGLMatrix lastKnownLocation) {
        float[] result = {lastKnownLocation.get(0, 3), lastKnownLocation.get(2, 3), lastKnownLocation.get(1, 3)};
        return result;
    }

    public float[] getRotation(OpenGLMatrix lastKnownLocation) {
        float[] result = {lastKnownLocation.get(1, 2), lastKnownLocation.get(1, 1), lastKnownLocation.get(0, 2)};
        return result;
    }
}