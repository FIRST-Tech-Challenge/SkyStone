/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/* Code taken from ConceptVuforiaSkyStoneNavigationWebcam
 */
/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 * <p>
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 * <p>
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.
 * <p>
 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 * <p>
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

package org.firstinspires.ftc.teamcode.SubAssembly.Vucam;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


public class VucamControl {
    /* Declare private class object */
    private boolean debugcode = false;
    private LinearOpMode opmode = null;     /* local copy of opmode class */
    private ElapsedTime runtime = new ElapsedTime();

    // **********************************************
    // Begin ConceptVuforiaSkyStoneNavigationWebcam

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

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
            "AU3bRpT/////AAABmXojhhSQw0t1v5y+m3h8AKkWsNNBNdx9pchrAjkbpsDQzCf6qEeBQ52HTxfNpr573118aw6sGlJT04GBul7QkXGdnghzUmFXconiUSC6bn3OOYXXlHsrF9r7czfAWv0Xab7R7ho+K+mS+l2o4oyfxxEu7LttxCIR4J5nDRcIA07LeTVwun2dvnL4xtufk00tOuO7DKQhmDc0iSY8hzi2DgDCYmn7PAvWBmgxUc3pSxbKE4Q8P0nn/S/2WVDiBmOon3mS5LWnNHQqobCmnoaR8zRZYKbPZ2gXE+uVi9XFtWeH63irNEMn0U/u4SwmNHPJfNG2j4Y7Qt2zMO+Vvg9s9ILLUlQRiGRePrl4bEHnORai";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    VuforiaTrackable stoneTarget;
    VuforiaTrackables targetsSkyStone;

    // End ConceptVuforiaSkyStoneNavigationWebcam
    // **********************************************

    public enum SkystonePosition {
        INIT, LEFT, CENTER, RIGHT
    }

    public SkystonePosition Skystone = SkystonePosition.INIT;

    /* Subassembly constructor */
    public VucamControl() {
    }

    public void init(LinearOpMode opMode) {
        init(opMode, false);
    }

    public void init(LinearOpMode opMode, boolean debug) {
        HardwareMap hwMap;

        debugcode = debug;

        opMode.telemetry.addLine("Vucam Control" + " initialize");
        opMode.telemetry.update();

        /* Set local copies from opmode class */
        opmode = opMode;
        hwMap = opMode.hardwareMap;

        if (debugcode)
            return;

        // **********************************************
        // Begin ConceptVuforiaSkyStoneNavigationWebcam

        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hwMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        // End ConceptVuforiaSkyStoneNavigationWebcam
        // **********************************************

    }

    public void updateTarget() {
        if (debugcode)
            return;

        // default to LEFT if we do not see anything
        targetVisible = false;
        Skystone = SkystonePosition.LEFT;

        // check if stone is visible.
        if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
            // getUpdatedRobotLocation() will return null if no new information is available since
            // the last time that call was made, or if the trackable is not currently visible.
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }

            VectorF translation = lastLocation.getTranslation();
            double yposition = translation.get(1);

            final double RIGHT_CENTER_THRESHOLD_INCHES = 0.0;
            if (yposition / mmPerInch > RIGHT_CENTER_THRESHOLD_INCHES) {
                Skystone = SkystonePosition.RIGHT;
            } else {
                Skystone = SkystonePosition.CENTER;
            }
            targetVisible = true;
        }
    }

    public void findTarget(double timeoutSEC) {
        if (debugcode)
            return;

        // if we already found target, then return
        if (targetVisible)
            return;

        double startTime = 0;
        double elapsedTime = 0;
        startTime = runtime.seconds();
        do {
            updateTarget();
            elapsedTime = runtime.seconds() - startTime;
            opmode.sleep(40);
        } while ( !targetVisible && (elapsedTime < timeoutSEC) && !opmode.isStopRequested());
    }

    public void Start() {
        if (debugcode)
            return;

        targetsSkyStone.activate();
    }

    public void Stop() {
        if (debugcode)
            return;

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
    }

    public void Telemetry() {
        if (debugcode)
            return;

        switch (Skystone) {
            case INIT:
                opmode.telemetry.addLine("Initial");
                break;
            case LEFT:
                opmode.telemetry.addLine("Left");
                break;
            case CENTER:
                opmode.telemetry.addLine("Center");
                break;
            case RIGHT:
                opmode.telemetry.addLine("Right");
                break;
        }

        if (targetVisible) {
            VectorF translation = lastLocation.getTranslation();
            opmode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            opmode.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
    }
}
