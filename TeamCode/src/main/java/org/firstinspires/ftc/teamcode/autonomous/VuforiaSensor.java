package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

@Autonomous(name = "VuforiaSensorClass")
public class VuforiaSensor {

    public VuforiaLocalizer vuforiaLocalizer;

    public VuforiaLocalizer.Parameters parameters;

    public VuforiaTrackables visionTargets;

    public VuforiaTrackable targetBluePerimeter;
    public VuforiaTrackable targetRedPerimeter;
    public VuforiaTrackable targetFrontPerimeter;
    public VuforiaTrackable targetBackPerimeter;
//    public VuforiaTrackable targetRedPerimeterTgt1;
//    public VuforiaTrackable targetRedPerimeterTgt2;
//    public VuforiaTrackable targetFrontPerimeterTgt1;
//    public VuforiaTrackable targetFrontPerimeterTgt2;
//    public VuforiaTrackable targetBluePerimeterTgt1;
//    public VuforiaTrackable targetBluePerimeterTgt2;
//    public VuforiaTrackable targetRearPerimeterTgt1;
//    public VuforiaTrackable targetRearPerimeterTgt2;


    public VuforiaTrackableDefaultListener listenerBluePerimeter;
    public VuforiaTrackableDefaultListener listenerRedPerimeter;
    public VuforiaTrackableDefaultListener listenerFrontPerimeter;
    public VuforiaTrackableDefaultListener listenerBackPerimeter;

    public OpenGLMatrix lastKnownLocationBluePerimeter;
    public OpenGLMatrix lastKnownLocationRedPerimeter;
    public OpenGLMatrix lastKnownLocationFrontPerimeter;
    public OpenGLMatrix lastKnownLocationBackPerimeter;

    public OpenGLMatrix phoneLocation;

    public VuforiaSensor() {
        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = "ATUNNu//////AAABmU6BPERoN0USgSQzxPQZ8JYg9RVnQhKO6YEHbNnOhkfL/iNrji3x9vzFkKsBgVzWgwH72G6eXpb3VCllKTrt1cD3gvQXZ48f+5EN43eYUQ3nuP3943NZB822XzV1djS3s6wDdaiS20PErO5K7lZUGyf9Z4Tb2TliOXv/ZoxUvwNQ/ndRjN344G0TAo8PUja0V3x2WKk+mCJavoZIgmOqgaitgmg5jim/aWBL2yk0a/QpqbP87KQfGn69zpisDBc98xdGPdSFj9ENkU9WTMem9UgnOFPgpdrHV5Zr5IpQH1jxLZIvwGuKOT97npm54kIvnJM0dzhBVA+s95JA3cxyac5ArHUYVtDePwlExuekZy9l"; // Insert your own key here
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("RoverRuckus");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        // Setup the target to be tracked
        targetBluePerimeter = visionTargets.get(0); // 0 corresponds to the BluePerimeter target
        targetBluePerimeter.setName("BluePerimeter");
        targetBluePerimeter.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        targetRedPerimeter = visionTargets.get(1);
        targetRedPerimeter.setName("RedPerimeter");
        targetRedPerimeter.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        targetFrontPerimeter = visionTargets.get(2);
        targetFrontPerimeter.setName("FrontPerimeter");
        targetFrontPerimeter.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        targetBackPerimeter = visionTargets.get(3);
        targetBackPerimeter.setName("BackPerimeter");
        targetBackPerimeter.setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        //Set phone location on robot
        phoneLocation = createMatrix(0, 0, 0, 0, 0, 0);

        // Setup listener and inform it of phone information
        listenerBluePerimeter = (VuforiaTrackableDefaultListener) targetBluePerimeter.getListener();
        listenerBluePerimeter.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        listenerRedPerimeter = (VuforiaTrackableDefaultListener) targetRedPerimeter.getListener();
        listenerRedPerimeter.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        listenerFrontPerimeter = (VuforiaTrackableDefaultListener) targetFrontPerimeter.getListener();
        listenerFrontPerimeter.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        listenerBackPerimeter = (VuforiaTrackableDefaultListener) targetBackPerimeter.getListener();
        listenerBackPerimeter.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        lastKnownLocationBluePerimeter = createMatrix(0, 0, 0, 0, 0, 0);
        lastKnownLocationRedPerimeter = createMatrix(0, 0, 0, 0, 0, 0);
        lastKnownLocationFrontPerimeter = createMatrix(0, 0, 0, 0, 0, 0);
        lastKnownLocationBackPerimeter = createMatrix(0, 0, 0, 0, 0, 0);
    }

    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    private String formatMatrix(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }

}